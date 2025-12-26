#include "PositionUpdateReplyModule.h"

#include <cctype>
#define _USE_MATH_DEFINES
#include <cmath>
#include <cassert>
#include <cstring>

#include <string>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <initializer_list>

#include "GPSStatus.h"
#include "GeoCoord.h"
#include "MeshService.h"

#include "DirectMessageReplyModule.h" // For reply_utils::... functions

using namespace reply_utils;

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

    return config;
}

void PositionUpdateReplyModule::setDefault() {
    moduleConfig.position_update_reply = getDefaultConfig();
}

size_t PositionUpdateReplyModule::getCodeWord(uint32_t const source, std::string &codeWord) const {
    auto const &config = moduleConfig.position_update_reply;
    auto const  iter   = m_monitored.find(source);
    auto const  index  = (iter == m_monitored.end() ? 0 : iter->second);
    getIthValue(config.start_code_word, codeWord, index);
    return index;
}

bool PositionUpdateReplyModule::hasNextNode(ConfigType const &config) {
    return !std::string(config.next_node).empty() || !std::string(config.next_code_word).empty();
}

PositionUpdateReplyModule::PositionUpdateReplyModule()
: MultiPortModule(
        "positionupdatereply"
        , std::initializer_list<decltype(meshtastic_PortNum_TEXT_MESSAGE_APP)>{
            meshtastic_PortNum_TEXT_MESSAGE_APP
            , meshtastic_PortNum_POSITION_APP
        }
) {
    LOG_DEBUG("PositionUpdateReplyModule constructor");
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

    float ilatLonToFloat(int32_t const ival) { return ival * 1e-7; }

    std::string geoCoordToString(GeoCoord const &val, uint32_t const nsats = 0) {
        return std::to_string( ilatLonToFloat(val.getLatitude()))
            + ", " + std::to_string(ilatLonToFloat(val.getLongitude()))
            + ", " + std::to_string(val.getAltitude()) + "m"
            + std::string{nsats == 0 ? "" : " " + std::to_string(nsats) + "s"}
        ;
    }

    /// Normalize bearing to [0, 360)
    float normalizeBearing(float const bearing) {
        float normalized = std::fmod(bearing, 360.0f);
        if (normalized < 0.0f) {
            normalized += 360.0f;
        }
        return normalized;
    }
} // namespace anonymous

ProcessMessage PositionUpdateReplyModule::handleReceivedTextMessage(const meshtastic_MeshPacket &mp) {
    if(mp.decoded.portnum != meshtastic_PortNum_TEXT_MESSAGE_APP)
        return ProcessMessage::CONTINUE;

    auto        const &config = moduleConfig.position_update_reply;
    auto        const &p      = mp.decoded;
    auto        const &source = (p.source ? p.source : mp.from); // Does this always come from mp?
    std::string const message{reinterpret_cast<const char *>(p.payload.bytes), p.payload.size};

    std::string       codeWord;
    size_t      const icodeWord    = getCodeWord(source, codeWord);
    bool        const isCodeWord   = !equalIgnoreCase(codeWord, "start");
    bool        const haveNextNode = hasNextNode(config);

    if(equalIgnoreCase(message, codeWord)) {
        if(!isCodeWord) LOG_DEBUG("Starting monitoring node: %u", source);
        else            LOG_DEBUG("Received codeWord: %u, from node: %u", icodeWord, source);

        m_monitored[source] = icodeWord+1;

        std::string response = R"(Position update replys enabled.
Will respond to regular updates or "exchange position" requests.
Send "stop" to disable.)";

        addToResponseIf(haveNextNode, response, "For clues about the next node, send a position update from within "
            + toStringPrecision(1, config.next_node_distance) + "m of this node.");

        sendReply(mp, response);
        return ProcessMessage::CONTINUE;
    }

    if(equalIgnoreCase(message, "stop")) {
        LOG_DEBUG("Stopping monitoring node: %u", source);
        m_monitored.erase(source);
        sendReply(mp, "Position update replys disabled. Send \"" + (isCodeWord ? std::string{"<codeword>"} : codeWord) + "\" to enable.");
        return ProcessMessage::CONTINUE;
    }

    if(equalIgnoreCase(message, "status")) {
        bool const trackingSender = (m_monitored.count(source) > 0);
        auto const num = m_monitored.size();
        std::string response = "Position update replys enabled for " + std::to_string(num) + (num == 1 ? " node" : " nodes");

        addToResponse(response, (trackingSender ? " including \"" : " not including \"") + getNodeShortName(source) + "\"");
        /// Add next node distance if next node info set
        addToResponseIf(haveNextNode,  response, "\nNext node distance: " + toStringPrecision(1, config.next_node_distance));
        addToResponseIf(icodeWord > 0, response, "\nCode word index: " + std::to_string(icodeWord));

        sendReply(mp, response);
        return ProcessMessage::CONTINUE;
    }

    return ProcessMessage::CONTINUE;
}

ProcessMessage PositionUpdateReplyModule::handleReceivedPosition(const meshtastic_MeshPacket &mp) {
    if(mp.decoded.portnum != meshtastic_PortNum_POSITION_APP)
        return ProcessMessage::CONTINUE;

    auto const &config = moduleConfig.position_update_reply;
    auto const &p      = mp.decoded;
    auto const &source = (p.source ? p.source : mp.from); // Does this always come from mp?

    if(p.reply_id != 0) {
        LOG_DEBUG("Skipping reply to message ID %u from %u", p.reply_id, source);
        return ProcessMessage::CONTINUE;
    }

    if(m_monitored.count(source) == 0)
        return ProcessMessage::CONTINUE;

    if (mp.which_payload_variant != meshtastic_MeshPacket_decoded_tag) {
        LOG_DEBUG("Skipping non-decoded payload with tag: %u", mp.which_payload_variant);
        return ProcessMessage::CONTINUE;
    }

    meshtastic_Position pos;
    memset(&pos, 0, sizeof(pos));
    if(!pb_decode_from_bytes(p.payload.bytes, p.payload.size, &meshtastic_Position_msg, &pos)) {
        LOG_ERROR("Error decoding position protobuf!");
        return ProcessMessage::STOP;
    }

    GeoCoord remote(pos.latitude_i, pos.longitude_i, pos.altitude);
    GeoCoord local(gpsStatus->getLatitude(), gpsStatus->getLongitude(), gpsStatus->getAltitude());

    auto const remoteSats      = pos.sats_in_view;
    auto const localSats       = gpsStatus->getNumSatellites();

    bool const haveLoc         = (local.getLatitude() != 0 || local.getLongitude() != 0 || local.getAltitude() != 0);
    auto const remotePrecision = precisionBitsToMeters(pos.precision_bits);
    auto const distance        = remote.distanceTo(local);
    auto const height          = local.getAltitude() - remote.getAltitude();
    auto const declination     = config.declination;
    bool const haveDecl        = (declination != 0.f);
    auto const trueBearing     = normalizeBearing((180.f / M_PI) * remote.bearingTo(local));
    auto       magBearing      = normalizeBearing(trueBearing - declination);

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
    _addToResponseIf(true, getNodeShortName(source) + ": " + geoCoordToString(remote, remoteSats));
    _addToResponseIf(true, " +- " + toStringPrecision(1, remotePrecision) + "m " + std::to_string(pos.precision_bits) + "bits", "\n");

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

    /// Mesh packet and signal metrics
    _addToResponseIf(config.send_hops,           "Hops: " + std::to_string(mp.hop_start) + "/" + std::to_string(mp.hop_limit), "\n");
    _addToResponseIf(config.send_signal_metrics, "SNR: "  + toStringPrecision(1, mp.rx_snr)); // + " dB";
    _addToResponseIf(config.send_signal_metrics, "RSSI: " + std::to_string(mp.rx_rssi));     // + " dBm";

    /// If set, reveal info about next node in sequence
    if(distance <= nextNodeDist) {
        if(remotePrecision > nextNodeDist) {
            _addToResponseIf(haveNextNode, "Unable to reveal clue due to bad precision", "\n");
        } else {
            std::string nextNode     = config.next_node;
            std::string nextCodeWord = config.next_code_word;

            auto numNodes     = getNumValues(config.next_node);
            auto numCodeWords = getNumValues(config.next_code_word);

            if(numNodes > 0 && numCodeWords > 0 && numNodes != numCodeWords) {
                LOG_WARN("PositionUpdateReplyModule: next_node has %zu entries, but next_code_word has %zu entries!", numNodes, numCodeWords);
            }

            if(numNodes     > 1) getIthValue(config.next_node,      nextNode,     (source % numNodes));
            if(numCodeWords > 1) getIthValue(config.next_code_word, nextCodeWord, (source % numCodeWords));

            _addToResponseIf(!nextNode.empty(),     "Next node: "     + nextNode,     "\n");
            _addToResponseIf(!nextCodeWord.empty(), "Next codeword: " + nextCodeWord, "\n");
        }
    }

    sendReply(mp, response);

    return ProcessMessage::CONTINUE;
}

ProcessMessage PositionUpdateReplyModule::handleReceived(const meshtastic_MeshPacket &mp) {
    auto const &config = moduleConfig.position_update_reply;
    auto const &p      = mp.decoded;

    if (!config.enabled) {
        LOG_DEBUG("PositionUpdateReplyModule is disabled, ignoring message");
        return ProcessMessage::CONTINUE;
    }

    switch(p.portnum) {
        case meshtastic_PortNum_TEXT_MESSAGE_APP: return handleReceivedTextMessage(mp);
        case meshtastic_PortNum_POSITION_APP:     return handleReceivedPosition(   mp);
        default:
            LOG_DEBUG("PositionUpdateReplyModule ignoring packet on port %d", p.portnum);
            break;
    }

    return ProcessMessage::CONTINUE;
}

void PositionUpdateReplyModule::sendReply(
    const   meshtastic_MeshPacket &mp
    , const std::string           &response
) {
    auto const &p      = mp.decoded;
    auto const &source = (p.source ? p.source : mp.from); // Does this always come from mp?
    auto        reply  = allocDataPacket();

    reply->to = source; // Reply to the source of the original message

    assert(reply->from == nodeDB->getNodeNum()); // Should always be our node number

    reply->decoded.reply_id = mp.id; // Set the reply ID to the original message ID

    copyStringToPayload(reply->decoded.payload, response);

    LOG_DEBUG("Replying to %u with: %s", source, response.c_str());

    service->sendToMesh(reply, RX_SRC_LOCAL);
}
