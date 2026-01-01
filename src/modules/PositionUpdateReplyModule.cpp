#include "PositionUpdateReplyModule.h"

#include <cctype>
#define _USE_MATH_DEFINES
#include <cmath>
#include <cassert>
#include <cstring>
#include <cstdlib>
#include <cctype>

#include <string>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <initializer_list>

#include "GPSStatus.h"
#include "GeoCoord.h"
#include "MeshService.h"

#include "DirectMessageReplyModule.h" // For reply_utils::... functions
#include "DebugConfiguration.h"
#include "ReplyUtils.h"

#ifndef LOG_PREFIX_PUR
/// Make messages cyan and add module specific prefix
//#   define LOG_PREFIX_PUR "\u001b[36m" "[PosUpRep] " // Doesn't work. maybe due to use of vsnprintf in the logger?
#   define LOG_PREFIX_PUR "[PosUpRep] "
#endif

#ifndef LOG_PREFIX
#   define LOG_PREFIX LOG_PREFIX_PUR
#endif

#include "LogUtils.h"

using namespace reply_utils;

PositionUpdateReplyModule::ConfigType &PositionUpdateReplyModule::getConfig() {
    return moduleConfig.position_update_reply;
}

/// The config.next_node and config.next_code_word can contain either a single
/// next node/code word pair, or a '|' delimited list of nodes. If more than one
/// pair is set, the address of the sender will be used to determine which pair
/// is sent in the reply. This is intended to be used so that multiple different
/// "courses" of nodes can be setup concurrently.
PositionUpdateReplyModule::ConfigType PositionUpdateReplyModule::getDefaultConfig() {
    ConfigType config;

    config.enabled             = true;
    config.send_location       = false;
    config.send_distance       = true;
    config.send_bearing        = true;
    config.send_hops           = true;
    config.send_signal_metrics = true;
    config.declination         = 0.f;

    /// If not set, code will replace with "start"
    //strncpy(config.start_code_word, "", sizeof(config.start_code_word));
    //config.start_code_word[sizeof(config.start_code_word) - 1] = '\0'; // Ensure null-termination
    config.start_code_word[0] = '\0';

    /// Defaults to empty
    //strncpy(config.next_code_word, "", sizeof(config.next_code_word));
    //config.next_code_word[sizeof(config.next_code_word) - 1] = '\0'; // Ensure null-termination
    config.next_code_word[0] = '\0';

    /// Defaults to empty
    //strncpy(config.next_node, "", sizeof(config.next_node));
    //config.next_node[sizeof(config.next_node) - 1] = '\0'; // Ensure null-termination
    config.next_node[0] = '\0';

    /// Default next node distance is 4 meters
    config.next_node_distance = 4.f;

    config.lat_lons[0] = '\0';
    config.bearing_offset = 0.f;

    return config;
}

void PositionUpdateReplyModule::setDefault() {
    getConfig() = getDefaultConfig();
}


PositionUpdateReplyModule::PositionUpdateReplyModule()
: MultiPortModule(
        "positionupdatereply"
        , std::initializer_list<decltype(meshtastic_PortNum_TEXT_MESSAGE_APP)>{
            meshtastic_PortNum_TEXT_MESSAGE_APP
            , meshtastic_PortNum_POSITION_APP
        }
) {
    LOG_DEBUG_PFX("PositionUpdateReplyModule constructor");
}

namespace {
    float precisionBitsToMeters(int const bits) {
        if(bits == 0.f) {
            return 0.f; // No precision, return 0
        }
        // Convert precision bits to meters
        // 23905787.925008 is the factor for converting bits to meters
        // 0.5^bits gives the precision in meters
        return static_cast<float>(23905787.925008f * std::pow(0.5f, bits));
    }

    std::string geoCoordToString(GeoCoord const &val, uint32_t const nsats = 0) {
        return std::to_string(      val.getLatAsDouble())
            + ", " + std::to_string(val.getLonAsDouble())
            + ", " + std::to_string(val.getAltitude()) + "m"
            + std::string{nsats == 0 ? "" : " " + std::to_string(nsats) + "s"}
        ;
    }

    inline float deg_to_rad(float const degrees) { return DEG_CONVERT*degrees; }
    inline float rad_to_deg(float const radians) { return radians/DEG_CONVERT; }

    /// Normalize bearing to [0, 360)
    inline float normalizeBearing(float const bearing, float const maxBearing = 360.f) {
        float normalized = std::fmod(bearing, maxBearing);
        if(normalized < 0.f) normalized += maxBearing;
        return normalized;
    }

    /// Normalize bearing to [0, 2*PI)
    inline float normalizeBearingRad(float const bearing) { return normalizeBearing(bearing, 2*PI); }
} // namespace anonymous

ProcessMessage PositionUpdateReplyModule::handleReceivedTextMessage(const meshtastic_MeshPacket &mp) {
    if(mp.decoded.portnum != meshtastic_PortNum_TEXT_MESSAGE_APP)
        return ProcessMessage::CONTINUE;

    auto        const &config = getConfig();
    auto        const &p      = mp.decoded;
    auto        const &source = (p.source ? p.source : mp.from); // Does this always come from mp?
    std::string const message{reinterpret_cast<const char *>(p.payload.bytes), p.payload.size};
    bool        const haveNextNode = hasNextNode(config) || (numCodeWords(config) > 1);

    size_t index = 0;
    bool const monitored = getSourceIndex(source, index);

    if(matchesCodeWord(message, index)) {
        bool const isCodeWord = !equalIgnoreCase(message, "start");

        if(!isCodeWord) LOG_DEBUG_PFX("Starting monitoring node: %u", source);
        else            LOG_DEBUG_PFX("Received codeWord: %u, from node: %u", index, source);

        m_monitored[source] = index;

        std::string response = R"(Position update replys enabled.
Will respond to regular updates or "exchange position" requests.
Send "stop" to disable.)";

        addToResponseIf(haveNextNode, response, "For clues about the next node, send a position update from within "
            + toStringPrecision(1, config.next_node_distance) + "m of this node.");

        sendReply(mp, response);
        return ProcessMessage::STOP;
    }

    if(equalIgnoreCase(message, "stop")) {
        bool isCodeWord = true;
        std::string codeWord;
        if(getCodeWord(index, codeWord))
            isCodeWord = !equalIgnoreCase(codeWord, "start");

        LOG_DEBUG_PFX("Stopping monitoring node: %u", source);
        m_monitored.erase(source);
        sendReply(mp, "Position update replys disabled. Send \"" + (isCodeWord ? std::string{"<codeword>"} : "start") + "\" to enable.");
        return ProcessMessage::STOP;
    }

    if(equalIgnoreCase(message, "status")) {
        bool const trackingSender = (m_monitored.count(source) > 0);
        auto const num = m_monitored.size();
        std::string response = "Position update replys enabled for " + std::to_string(num) + (num == 1 ? " node" : " nodes");

        addToResponse(response, (trackingSender ? "including \"" : "not including \"") + getNodeShortName(source) + "\"");
        /// Add next node distance if next node info set
        addToResponseIf(haveNextNode,           response, "Next node distance: " + toStringPrecision(1, config.next_node_distance), "\n");
        addToResponseIf(monitored && index > 0, response, "Code word index: "    + std::to_string(index), "\n");

        sendReply(mp, response);
        return ProcessMessage::STOP;
    }

    return ProcessMessage::CONTINUE;
}

ProcessMessage PositionUpdateReplyModule::handleReceivedPosition(const meshtastic_MeshPacket &mp) {
    static size_t constexpr max_next_node_length = 12;

    if(mp.decoded.portnum != meshtastic_PortNum_POSITION_APP)
        return ProcessMessage::CONTINUE;

    auto const &config = getConfig();
    auto const &p      = mp.decoded;
    auto const &source = (p.source ? p.source : mp.from); // Does this always come from mp?

    if(p.reply_id != 0) {
        LOG_DEBUG_PFX("Skipping reply to message ID %u from %u", p.reply_id, source);
        return ProcessMessage::CONTINUE;
    }

    if(m_monitored.count(source) == 0)
        return ProcessMessage::CONTINUE;

    if (mp.which_payload_variant != meshtastic_MeshPacket_decoded_tag) {
        LOG_DEBUG_PFX("Skipping non-decoded payload with tag: %u", mp.which_payload_variant);
        return ProcessMessage::CONTINUE;
    }

    meshtastic_Position pos;
    memset(&pos, 0, sizeof(pos));
    if(!pb_decode_from_bytes(p.payload.bytes, p.payload.size, &meshtastic_Position_msg, &pos)) {
        LOG_ERROR_PFX("Error decoding position protobuf!");
        return ProcessMessage::STOP;
    }

    GeoCoord       remote(pos.latitude_i, pos.longitude_i, pos.altitude);
    GeoCoord const local = getLocalGeoCoord(source);

    auto const remoteSats      = pos.sats_in_view;
    auto const localSats       = gpsStatus->getNumSatellites();

    bool const haveLoc         = (local.getLatitude() != 0 || local.getLongitude() != 0 || local.getAltitude() != 0);
    auto const remotePrecision = precisionBitsToMeters(pos.precision_bits);
    auto const distance        = remote.distanceTo(local);
    auto const height          = local.getAltitude() - remote.getAltitude();
    auto const declination     = config.declination;
    bool const haveDecl        = (declination != 0.f);
    auto const trueBearing     = normalizeBearing(rad_to_deg(remote.bearingTo(local)));
    auto const magBearing      = normalizeBearing(trueBearing - declination);

    assert(trueBearing >= 0.f && trueBearing < 360.f);
    assert(magBearing  >= 0.f && magBearing  < 360.f);

    std::string response;

    // Check if we need a separator and add string to response
    auto _addToResponse = [&response](std::string const &str, char const *sep = " ") {
        addToResponse(response, str, sep);
    };

    // Add to response if condition is true
    auto _addToResponseIf = [&response](bool const condition, std::string const &str, char const *sep = " ") {
        addToResponseIf(condition, response, str, sep);
    };

    /// Next node info
    auto const &nextNodeDist = config.next_node_distance;
    bool const  haveNextNode = hasNextNode(config);

    /// If next node, and position is manually set, reject!
    if(haveNextNode && pos.location_source == meshtastic_Position_LocSource_LOC_MANUAL) {
        _addToResponse("Cheater!");
        sendReply(mp, response);
        return ProcessMessage::CONTINUE;
    }

    /// Received GPS info
    _addToResponseIf(true, getNodeShortName(source) + ": " + geoCoordToString(remote, remoteSats), "\n");
    _addToResponseIf(true, "+- " + toStringPrecision(1, remotePrecision) + "m " + std::to_string(pos.precision_bits) + "bits");

    /// Local GPS info + distance and bearing between remote and local
    if(!haveLoc) {
        bool const needLoc = (config.send_location || config.send_distance || config.send_bearing);
        _addToResponseIf(needLoc, getNodeShortName() + ": " + "LLA not available", "\n");
    } else {
        _addToResponseIf(config.send_location, getNodeShortName() + ":  " + geoCoordToString(local, localSats), "\n");
        _addToResponseIf(config.send_distance,            "Dist: "   + toStringPrecision(1, distance) + "m","\n");
        _addToResponseIf(config.send_distance,            "Height: " + toStringPrecision(1, height  ) + "m","\n");
        _addToResponseIf(config.send_bearing,             "True: "   + toStringPrecision(1, trueBearing), "\n");
        _addToResponseIf(config.send_bearing && haveDecl, "Mag: "    + toStringPrecision(1, magBearing ), "\n");
        _addToResponseIf(config.send_bearing && haveDecl, "Decl: "   + toStringPrecision(1, declination), "\n");
    }

    /// If set, reveal info about next node in sequence
    if(distance <= nextNodeDist) {
        if(remotePrecision > nextNodeDist) {
            _addToResponseIf(haveNextNode, "Unable to reveal clue due to bad precision", "\n");
        } else {
            std::string nextNode;
            std::string nextCodeWord;

            getNextNodeCode(source, nextNode, nextCodeWord);

            _addToResponseIf(!nextNode.empty(),     "Next node: "     + nextNode.substr(0, max_next_node_length), "\n");
            _addToResponseIf(!nextCodeWord.empty(), "Next codeword: " + nextCodeWord, "\n");
        }
    } else {
        /// Mesh packet and signal metrics
        _addToResponseIf(config.send_hops,           "Hops: " + std::to_string(mp.hop_start) + "/" + std::to_string(mp.hop_limit), "\n");
        _addToResponseIf(config.send_signal_metrics, "SNR: "  + toStringPrecision(1, mp.rx_snr)); // + " dB";
        _addToResponseIf(config.send_signal_metrics, "RSSI: " + std::to_string(mp.rx_rssi));     // + " dBm";
    }

    sendReply(mp, response);

    return ProcessMessage::CONTINUE;
}

ProcessMessage PositionUpdateReplyModule::handleReceived(const meshtastic_MeshPacket &mp) {
    auto const &config = getConfig();
    auto const &p      = mp.decoded;

    if (!config.enabled) {
        LOG_DEBUG_PFX("PositionUpdateReplyModule is disabled, ignoring message");
        return ProcessMessage::CONTINUE;
    }

    switch(p.portnum) {
        case meshtastic_PortNum_TEXT_MESSAGE_APP: return handleReceivedTextMessage(mp);
        case meshtastic_PortNum_POSITION_APP:     return handleReceivedPosition(   mp);
        default:
            LOG_DEBUG_PFX("PositionUpdateReplyModule ignoring packet on port %d", p.portnum);
            break;
    }

    return ProcessMessage::CONTINUE;
}

void PositionUpdateReplyModule::sendReply(
    const   meshtastic_MeshPacket &mp
    , const std::string           &response
    , const size_t                 maxLength
) {
    auto const &p      = mp.decoded;
    auto const &source = (p.source ? p.source : mp.from); // Does this always come from mp?
    auto        reply  = allocDataPacket();

    reply->to = source; // Reply to the source of the original message

    assert(reply->from == nodeDB->getNodeNum()); // Should always be our node number

    reply->decoded.reply_id = mp.id; // Set the reply ID to the original message ID

    copyStringToPayload(reply->decoded.payload, response.substr(0, maxLength)); // Truncate to max size

    LOG_DEBUG_PFX("Replying to %u with: %s", source, response.c_str());

    service->sendToMesh(reply, RX_SRC_LOCAL);
}

bool PositionUpdateReplyModule::hasLatLons(ConfigType const &config) {
    return !std::string(config.lat_lons).empty();
}

bool PositionUpdateReplyModule::hasNextNode(ConfigType const &config) {
    return !std::string(config.next_node).empty() || !std::string(config.next_code_word).empty();
}

size_t PositionUpdateReplyModule::numCodeWords(ConfigType const &config) {
    return getNumValues(config.start_code_word);
}

/// Check if message matches codeword
bool PositionUpdateReplyModule::matchesCodeWord(std::string const &message, size_t &icodeWord) const {
    auto        const &config       = getConfig();
    std::string const  codeWords    = config.start_code_word;
    auto        const  numCodeWords = getNumValues(codeWords);

    if(     numCodeWords == 0) { return equalIgnoreCase(message, "start"); }
    else if(numCodeWords == 1) { return equalIgnoreCase(message, codeWords); }

    for(size_t i = 0; i < numCodeWords; ++i) {
        std::string codeWord;
        if(!getIthValue(codeWords, codeWord, i)) {
            LOG_WARN_PFX("Unable to retrieve %luth codeword", i);
            continue;
        }
        if(equalIgnoreCase(message, codeWord) || (codeWord.empty() && equalIgnoreCase(message, "start"))) {
            icodeWord = i;
            return true;
        }
    }

    return false;
}

/// Return true if source is being monitored and retrieve current source index
bool PositionUpdateReplyModule::getSourceIndex(uint32_t const source, size_t &index) const {
    auto const iter  = m_monitored.find(source);
    bool const found = (iter != m_monitored.end());
    index = (!found ? 0 : iter->second);
    return found;
}

bool PositionUpdateReplyModule::getCodeWord(size_t const index, std::string &codeWord) const {
    return getIthValue(getConfig().start_code_word, codeWord, index);
}

/// Retrieve current position to report against for source
/// Source node is needed so that the index for the currently active code word for that
/// node can be looked up and the corresponding lat/lon retrieved.
GeoCoord PositionUpdateReplyModule::getLocalGeoCoord(uint32_t const source) const {
    auto const &config = getConfig();

    GeoCoord local(gpsStatus->getLatitude(), gpsStatus->getLongitude(), gpsStatus->getAltitude());

    size_t index = 0;
    if(!getSourceIndex(source, index)) {
        LOG_INFO_PFX("Source node %u not monitored, returning gps position", source);
        return local;
    }

    std::string lat_lon;
    if(!getIthValue(config.lat_lons, lat_lon, index)) {
        LOG_INFO_PFX("No lat/lon set for index %lu, returning gps position", index);
        return local;
    }

    if(lat_lon.empty()) {
        LOG_INFO_PFX("Empty lat/lon returning gps position");
        return local;
    }

    auto const pos = lat_lon.find(',');

    if(pos == std::string::npos) {
        LOG_ERROR_PFX("Invalid lat/lon, returning gps position");
        return local;
    }

    /// Trims last character if it is not a digit
    auto trimLastChar = [](std::string const &str) {
        if(str.empty()) return str;
        if(std::isdigit(str.back())) return str;
        return str.substr(0, str.size()-1);
    };

    auto const first     = lat_lon.substr(0, pos);
    auto const second    = lat_lon.substr(pos+1 );
    auto const firstVal  = std::atof(trimLastChar(first ).c_str());
    auto const secondVal = std::atof(trimLastChar(second).c_str());

    if(first.empty() || second.empty()) {
        LOG_ERROR_PFX("Missing first or second value in lat/lon pair, %s, %s", first.c_str(), second.c_str());
        return local;
    }

    if(std::isdigit(second.back())) {
        auto const lat = firstVal;
        auto const lon = secondVal;

        LOG_INFO_PFX("Using lat/lon %f, %f %.1f for index %lu", lat, lon, local.getAltitude(), index);

        local.updateCoords(lat, lon, local.getAltitude());
        return local;
    }

    /// If last char in second value is not a digit, values are either an easting and a northing
    /// or a bearing and a range relative to the node's gps position. Examples:

    ///  100,200m  =>  100 meter easting and  200 meter northing
    /// -100,200n  => -100 meter easting and  200 meter northing
    ///  100w,200n => -100 meter easting and  200 meter northing
    /// 100e,-200n =>  100 meter easting and -200 meter northing
    /// 100e,200s  =>  100 meter easting and -200 meter northing
    /// 270d,300m  =>  270 degree bearing and 300 meter range

    /// Returns true if lower(last char) matches lower(chr || chr2 || chr3)
    auto lastCharIs = [](std::string const &str, char const chr, char const chr2 = '\0', char const chr3 = '\0') {
        char const back = std::tolower(str.back());
        return !str.empty() && (back == std::tolower(chr) || back == std::tolower(chr2) || back == std::tolower(chr3));
    };

    // Last value is either in meters or a northing
    assert(lastCharIs(second, 'm', 'n', 's'));

    double bearing{};
    double range{};

    // First value is either in meters or an easting(if no char specifier, assume easting)
    if(lastCharIs(first, 'm', 'e', 'w') || std::isdigit(first.back())) {
        auto const easting  = firstVal  * (lastCharIs(first,  'w') ? -1 : 1);
        auto const northing = secondVal * (lastCharIs(second, 's') ? -1 : 1);
        bearing = std::atan2(easting, northing);
        range   = std::hypot(easting, northing);
    // First value is a bearing in degrees
    } else if(lastCharIs(first, 'd')) {
        bearing  = deg_to_rad(firstVal);
        range    = secondVal;
    }

    if(config.bearing_offset != 0.f) {
        auto const bearing_orig = bearing;
        bearing += deg_to_rad(config.bearing_offset);
        if(bearing <  0   ) bearing += 2*PI;
        if(bearing >= 2*PI) bearing -= 2*PI;
        LOG_INFO_PFX("Rotating bearing by %f degrees %f => %f", config.bearing_offset, rad_to_deg(bearing_orig), rad_to_deg(bearing));
    }

    auto local_new = *local.pointAtDistance(bearing, range);
    LOG_INFO_PFX("Using lat/lon %f, %f %.1f for index %lu bearing: %f range: %f"
        , local_new.getLatAsDouble()
        , local_new.getLonAsDouble()
        , local_new.getAltitude(), index, rad_to_deg(bearing), range
    );

    return local_new;
}

/// Retrieve next node and next code word in sequence
void PositionUpdateReplyModule::getNextNodeCode(uint32_t const source, std::string &nextNode, std::string &nextCodeWord) const {
    auto const &config           = getConfig();
    auto const  numCodeWords     = getNumValues(config.start_code_word);
    auto const  numNextNodes     = getNumValues(config.next_node);
    auto const  numNextCodeWords = getNumValues(config.next_code_word);

    if(numNextNodes > 0 && numNextCodeWords > 0 && numNextNodes != numNextCodeWords) {
        LOG_WARN_PFX("PositionUpdateReplyModule: next_node has %lu entries, but next_code_word has %lu entries!", numNextNodes, numNextCodeWords);
    }

    nextNode.clear();
    nextCodeWord.clear();

    /// If only one code word for this node, use (source % numNext) to lookup next node/code
    if(numCodeWords <= 1) {
        /// Retrieve (source % numNextNodes)th value
        getIthValueModNum(config.next_node,      nextNode,     source, numNextNodes    );
        getIthValueModNum(config.next_code_word, nextCodeWord, source, numNextCodeWords);

        LOG_INFO_PFX("Lookup next node by (source %% numNext): (%u, %lu) node: %s", source, numNextNodes,     nextNode.c_str());
        LOG_INFO_PFX("Lookup next code by (source %% numNext): (%u, %lu) code: %s", source, numNextCodeWords, nextCodeWord.c_str());
        return;
    }

    /// If multiple code words for this node, lookup by the index for the currently active code word for this source
    assert(numCodeWords > 1);

    size_t index = 0;
    getSourceIndex(source, index);

    if(numNextNodes     > 0) { getIthValueModNum(config.next_node, nextNode, index, numNextNodes); }
    else                     { nextNode = getNodeLongName(); }

    if(numNextCodeWords > 0) { getIthValueModNum(config.next_code_word, nextCodeWord, index, numNextCodeWords); }
    else { /// Next code words not set, retrieve code word from next in list of code words for this node
        assert(numNextCodeWords == 0);

        if(!getCodeWord(index+1, nextCodeWord)) {
            /// If not found, we are out of code words and have reached the end
            if(numNextNodes == 0)
                nextNode = "winner!";
        }
    }

    LOG_INFO_PFX("Lookup next node by active code word for source %u (index %% numNext): (%u, %lu) node: %s", source, index, numNextNodes,     nextNode.c_str());
    LOG_INFO_PFX("Lookup next code by active code word for source %u (index %% numNext): (%u, %lu) code: %s", source, index, numNextCodeWords, nextCodeWord.c_str());
    return;
}
