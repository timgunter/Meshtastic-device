#include "DirectMessageReplyModule.h"

#include <cassert>
#include <cstring>

#include <string>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <memory>

#include <algorithm>

#include "Meshservice.h"
#include "configuration.h"
#include "main.h"

using namespace reply_utils;

/// This module automatically replies to direct messages with pre-configured responses.
/// These responses can be configured so that different responses will be provided based
/// upon different "queries" in the received direct message.
/// Other options allow the response to contain other info like signal metrics, hop info, etc.
///
/// The config.queries and config.responses are '|' delimited lists of queries and responses.
/// If either queries or responses is empty, they are consider to contain zero entries.
/// Matching of queries to messages is case-insensitive. Queries and responses can be configured
/// as follows.
///
/// nqueries == 0 && nresponses == 0        => no custom message will be added to the response
/// nqueries == 0 && nresponses == 1        => always use the single response
/// nqueries == 1 && nresponses == 1        => if message matches query, use the single response
/// nqueries == 0 && nresponses >  1        => use (source_address % nresponses) to select response
/// nqueries == 1 && nresponses >  1        => if message matches query, use (source_address % nresponses)th response
/// nqueries >  1 && nresponses == nqueries => if message matches nth query, use nth response
/// nqueries >  1 && nresponses >  nqueries => same as above, but extra responses are ignored

DirectMessageReplyModule::ConfigType DirectMessageReplyModule::getDefaultConfig() {
    ConfigType config;

    config.enabled             = true;
    config.channel             = 0;
    config.send_hops           = true;
    config.send_signal_metrics = true;
    config.echo_message        = true;
    config.echo_user           = true;
    config.queries[0]          = '\0';
    strncpy(config.responses, "Thanks for the DM!\n", sizeof(config.responses));
    config.responses[sizeof(config.responses) - 1] = '\0'; // Ensure null-termination

    return config;
}

void DirectMessageReplyModule::setDefault() {
    moduleConfig.direct_message_reply = getDefaultConfig();
}

ProcessMessage DirectMessageReplyModule::handleReceived(const meshtastic_MeshPacket &mp) {
    auto const &config = moduleConfig.direct_message_reply;

    if (!config.enabled) {
        LOG_DEBUG("DirectMessageReplyModule is disabled, ignoring message");
        return ProcessMessage::CONTINUE;
    }

    auto        const &channel        = mp.channel;
    auto        const &p              = mp.decoded;
    auto        const &source         = (p.source ? p.source : mp.from); // Does this always come from mp?
    auto        const &dest           = (p.dest ? p.dest : mp.to);
    auto        const &myID           = nodeDB->getNodeNum();
    bool        const isDM            = (dest != 0);
    auto        const sourceShortName = getNodeShortName(source);
    std::string const message{reinterpret_cast<const char *>(p.payload.bytes), p.payload.size};

    if (p.reply_id != 0) {
        LOG_DEBUG("Skipping reply to message ID %u from %u", p.reply_id, source);
        return ProcessMessage::CONTINUE;
    }

    if (dest != myID) {
        LOG_DEBUG("Ignoring message not for us(%u) decoded dest: %u packet dest: %u", myID, p.dest, mp.to);
        return ProcessMessage::CONTINUE;
    }

    // See if message matches as query and provide appropriate response if so.
    auto const numQueries   = getNumValues(config.queries);
    auto const numResponses = getNumValues(config.responses);

    size_t     iresponse    = 0;
    if(numQueries > 1 && numQueries != numResponses) {
        LOG_WARN("DirectMessageReplyModule: number of queries (%zu) is > 1 and does not match number of responses (%zu)", numQueries, numResponses);
    }

    // If there are no queries, or only one query and it matches the message
    if(numQueries == 0 || (numQueries == 1 && equalIgnoreCase(message, config.queries))) {
        if(numResponses <= 1) {
            iresponse = 0;
            LOG_DEBUG("DirectMessageReplyModule: 0 or 1 queries and 1 response");
        } else { // 0 or 1 queries and multiple responses; choose response based on source address
            iresponse = (source % numResponses);
            LOG_DEBUG("DirectMessageReplyModule: 0 or 1 queries and >1 response responding with response %zu", iresponse);
        }
    } else if(numQueries > 1) {
        // If more than 1 query, see if any match the message, and use corresponding response
        if(findMatch(config.queries, message, iresponse)) {
            if(iresponse < numResponses) {
                LOG_DEBUG("DirectMessageReplyModule: matched query index %zu", iresponse);
            } else {
                LOG_ERROR("DirectMessageReplyModule: matched query index %zu has no corresponding response (only %zu responses) falling a back to 0", iresponse, numResponses);
            }
        } else { // Otherwise, use the first response
            iresponse = 0;
            LOG_DEBUG("DirectMessageReplyModule: no match found, using response 0");
        }
    }

    std::string response;

    if(iresponse >= numResponses) {
        response = "Invalid DirectMessageReplyModule configuration";
    } else {
        getIthValue(config.responses, response, iresponse);
    }

    // Add to response if condition is true
    auto _addToResponseIf = [&response](bool const condition, std::string const &str, char const *sep = " ") {
        addToResponseIf(condition, response, str, sep);
    };

    _addToResponseIf(config.echo_user,           "User: "    + sourceShortName, "\n");
    _addToResponseIf(!isDM,                      "Channel: " + std::string(channels.getName(channel)), "\n");
    _addToResponseIf(config.send_hops,           "Hop lim: " + std::to_string(mp.hop_limit) + "/" + std::to_string(mp.hop_start), "\n");
    _addToResponseIf(config.send_signal_metrics, "SNR: "     + toStringPrecision(1, mp.rx_snr)); // + " dB";
    _addToResponseIf(config.send_signal_metrics, "RSSI: "    + std::to_string(mp.rx_rssi));     // + " dBm";

    _addToResponseIf(config.echo_message, message, "\n");

    sendReply(mp, response);

    return ProcessMessage::CONTINUE;
}

void DirectMessageReplyModule::sendReply(
    const   meshtastic_MeshPacket &mp
    , const std::string           &response
) {
    auto const &config = moduleConfig.direct_message_reply;
    auto const &p      = mp.decoded;
    auto const &source = (p.source ? p.source : mp.from); // Does this always come from mp?
    auto        reply  = allocDataPacket();

    if (config.channel > 0 && isPrivateChannel(config.channel))
        reply->channel = config.channel;
    else
        reply->to = source; // Reply to the source of the original message

    assert(reply->from == nodeDB->getNodeNum()); // Should always be our node number

    reply->decoded.reply_id = mp.id; // Set the reply ID to the original message ID

    copyStringToPayload(reply->decoded.payload, response);

    LOG_DEBUG("Replying to %u with: %s", source, response.c_str());

    service->sendToMesh(reply, RX_SRC_LOCAL);
}

namespace reply_utils {
    // Convert float to string with specified precision
    std::string toStringPrecision(int const precision, float const value) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(precision) << value;
        return oss.str();
    }

    // Copy a string into the payload, ensuring it does not exceed the payload size
    void copyStringToPayload(meshtastic_Data_payload_t &payload, std::string const &str) {
        payload.size = std::min(str.size(), sizeof(payload.bytes));
        std::memcpy(payload.bytes, str.c_str(), payload.size);
        assert(payload.size <= sizeof(payload.bytes));
    }

    bool isPrivateChannel(ChannelIndex const channel) {
        auto const &info = channels.getByIndex(channel);
        if (info.role == meshtastic_Channel_Role_DISABLED) {
            return false;
        }
        if (!info.has_settings) {
            return false;
        }
        if (info.settings.psk.size <= 16) {
            return false;
        }
        return true;
    }

    std::string getNodeShortName(uint32_t const nodeNum) {
        auto const *node = nodeDB->getMeshNode(nodeNum);
        if (node && node->has_user) {
            return node->user.short_name;
        }
        return "Unk";
    }

    std::string getNodeLongName(uint32_t const nodeNum) {
        auto const *node = nodeDB->getMeshNode(nodeNum);
        if (node && node->has_user) {
            return node->user.long_name;
        }
        return "Unknown";
    }

    bool equalIgnoreCase(std::string const &left, std::string const &right) {
        if(left.size() != right.size())
            return false;
        return std::equal(left.begin(), left.end(), right.begin()
            , [](char const l, char const r) {
                return std::tolower(l) == std::tolower(r);
        });
    }

    /// Get substring using start and end positions(instead of start and length)
    std::string startToEnd(std::string const &str, std::string::size_type const start, std::string::size_type const end) {
        if(end == std::string::npos) return str.substr(start);
        if(end < start)              return {};
        else                         return str.substr(start, end - start);
    }

    bool atEnd(std::string::size_type const pos) {
        return pos == std::string::npos;
    }

    /// Set start to current end, and increment past delimiter
    void setStart(std::string::size_type &start, std::string::size_type const end) {
        start = end;
        if(atEnd(start)) return;
        ++start; // Delimiter is single character
    }

    /// Search a delimited string of values for a particular value. "i" is 0-based index to value if found
    bool findMatch(std::string const &values, std::string const &value, size_t &I, char const delim) {
        std::string::size_type start = 0;
        std::string::size_type end   = 0;
        for(size_t i = 0; i < values.size() && !atEnd(start); ++i, setStart(start, end)) {
            end = values.find(delim, start);

            if(equalIgnoreCase(value, startToEnd(values, start, end))) {
                I = i;
                return true;
            }
        }

        return false;
    }

    /// Count number of values in a delimited string
    size_t getNumValues(std::string const &values, char const delim) {
        if(values.empty()) return 0;
        return 1 + std::count(values.begin(), values.end(), delim);
    }

    /// Retrieve Ith(0-based) value from a delimited list of values
    bool getIthValue(std::string const &values, std::string &value, size_t const I, char const delim) {
        size_t i = 0;
        std::string::size_type start = 0;
        std::string::size_type end   = 0;
        for(i = 0; i <= I && !atEnd(start); ++i, setStart(start, end)) {
            end = values.find(delim, start);

            if(i == I) {
                value = startToEnd(values, start, end);
                return true;
            }
        }

        return false;
    }

    // Check if we need a separator and add string to response
    void addToResponse(std::string &response, std::string const &str, char const *sep) {
        if (str.empty()) return;
        if (!response.empty() && response.back() != '\n')
            response += sep;
        response += str;
    };

    // Add to response if condition is true
    void addToResponseIf(bool const condition, std::string &response, std::string const &str, char const *sep) {
        if (condition) addToResponse(response, str, sep);
    };
} // namespace reply_utils
