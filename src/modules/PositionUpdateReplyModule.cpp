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
    bool equalIgnoreCase(std::string const &left, std::string const &right) {
        if(left.size() != right.size())
            return false;
        return std::equal(left.begin(), left.end(), right.begin()
            , [](char const l, char const r) {
                return std::tolower(l) == std::tolower(r);
        });
    }

    // Convert float to string with specified precision
    std::string toStringPrecision(int const precision, float const value) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(precision) << value;
        return oss.str();
    };

    // Copy a string into the payload, ensuring it does not exceed the payload size
    void copyStringToPayload(meshtastic_Data_payload_t &payload, std::string const &str) {
        payload.size = std::min(str.size(), sizeof(payload.bytes));
        std::memcpy(payload.bytes, str.c_str(), payload.size);
        assert(payload.size <= sizeof(payload.bytes));
    };

    std::string getNodeShortName(uint32_t const nodeNum = nodeDB->getNodeNum()) {
        auto const *node = nodeDB->getMeshNode(nodeNum);
        if (node && node->has_user) {
            return node->user.short_name;
        }
        return "Unk";
    }

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

    std::string const startCodeWord = (0 == strnlen(config.start_code_word, sizeof(config.start_code_word)) ? "start" : config.start_code_word);
    bool        const isCodeWord    = !equalIgnoreCase(startCodeWord, "start");
    std::string const nextCodeWord  = config.next_code_word;
    std::string const nextNode      = config.next_node;

    if(equalIgnoreCase(message, startCodeWord)) {
        LOG_DEBUG("Starting monitoring node: %u", source);
        m_monitored.insert(source);
        sendReply(
            mp,
            R"(Position update replys enabled.
Will respond to regular updates, "exchange position", or "request position" requests. Recommend increasing your GPS reporting precision.
Send "stop" to disable.)"
        );

        if(!nextNode.empty() || !nextCodeWord.empty()) {
            sendReply(mp, "To receive clues about the next node, send a position update from within "
                + toStringPrecision(1, config.next_node_distance) + "m of this node."
            );
        }
        return ProcessMessage::CONTINUE;
    }

    if(equalIgnoreCase(message, "stop")) {
        LOG_DEBUG("Stopping monitoring node: %u", source);
        m_monitored.erase(source);
        sendReply(mp, "Position update replys disabled. Send \"" + (isCodeWord ? std::string{"<codeword>"} : startCodeWord) + "\" to enable.");
        return ProcessMessage::CONTINUE;
    }

    if(equalIgnoreCase(message, "status")) {
        bool const trackingSender = (m_monitored.count(source) > 0);
        auto const num = m_monitored.size();
        std::string response = "Position update replys enabled for " + std::to_string(num) + (num == 1 ? " node" : " nodes")
                             +  (trackingSender ? " including \"" : " not including \"") + getNodeShortName(source) + "\"";

        /// Add next node distance if next node info set
        if(!nextNode.empty() || nextCodeWord.empty()) {
            response += "\nNext node distance: " + toStringPrecision(1, config.next_node_distance);
        }

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
    auto const declination     = config.declination;
    bool const haveDecl        = (declination != 0.f);
    auto const trueBearing     = normalizeBearing((180.f / M_PI) * remote.bearingTo(local));
    auto       magBearing      = normalizeBearing(trueBearing - declination);

    assert(trueBearing >= 0.f && trueBearing < 360.f);
    assert(magBearing  >= 0.f && magBearing  < 360.f);

    std::string response;

    // Check if we need a separator and add string to response
    auto addToResponse = [&response](std::string const &str, char const *sep = " ") {
        if (str.empty()) return;
        if (!response.empty() && response.back() != '\n')
            response += sep;
        response += str;
    };

    // Add to response if condition is true
    auto addToResponseIf = [&addToResponse](bool const condition, std::string const &str, char const *sep = " ") {
        if (condition) addToResponse(str, sep);
    };

    /// Next node info
    auto        const &nextNodeDist = config.next_node_distance;
    std::string const  nextNode     = config.next_node;
    std::string const  nextCodeWord = config.next_code_word;
    bool        const  haveNextNode = (!nextNode.empty() || !nextCodeWord.empty());

    /// If next node, and position is manually set, reject!
    if(haveNextNode && pos.location_source == meshtastic_Position_LocSource_LOC_MANUAL) {
        addToResponse("Cheater!");
        sendReply(mp, response);
        return ProcessMessage::CONTINUE;
    }

    /// Received GPS info
    addToResponseIf(true, getNodeShortName(source) + ": " + toStringPrecision(1, remotePrecision) + "m precision bits: " + std::to_string(pos.precision_bits), "\n");
    addToResponseIf(true, getNodeShortName(source) + ": " + geoCoordToString(remote, remoteSats), "\n");

    /// Local GPS info + distance and bearing between remote and local
    if(!haveLoc) {
        bool const needLoc = (config.send_location || config.send_distance || config.send_bearing);
        addToResponseIf(needLoc, getNodeShortName() + ": " + "LLA not available", "\n");
    } else {
        addToResponseIf(config.send_location, getNodeShortName() + ": " + geoCoordToString(local, localSats), "\n");
        addToResponseIf(config.send_distance,            "Distance:     " + toStringPrecision(1, distance) + "m","\n");
        addToResponseIf(config.send_bearing,             "True bearing: " + toStringPrecision(1, trueBearing), "\n");
        addToResponseIf(config.send_bearing && haveDecl, "Magnetic:     " + toStringPrecision(1, magBearing ), "\n");
        addToResponseIf(config.send_bearing && haveDecl, "Declination:  " + toStringPrecision(1, declination), "\n");
    }

    /// Mesh packet and signal metrics
    addToResponseIf(config.send_hops,           "Hop lim: " + std::to_string(mp.hop_limit) + "/" + std::to_string(mp.hop_start), "\n");
    addToResponseIf(config.send_signal_metrics, "SNR: " + toStringPrecision(1, mp.rx_snr)); // + " dB";
    addToResponseIf(config.send_signal_metrics, "RSSI: " + std::to_string(mp.rx_rssi));     // + " dBm";

    /// If set, reveal info about next node in sequence
    if(distance <= nextNodeDist) {
        if(remotePrecision > nextNodeDist) {
            addToResponseIf(haveNextNode, "Unable to reveal next node due to bad precision", "\n");
        } else {
            addToResponseIf(!nextNode.empty(),     "Next node:     " + nextNode,     "\n");
            addToResponseIf(!nextCodeWord.empty(), "Next codeword: " + nextCodeWord, "\n");
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
