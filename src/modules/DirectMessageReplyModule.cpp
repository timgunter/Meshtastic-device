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
#include "ReplyUtils.h"

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

    if(iresponse >= numResponses && numResponses > 0) {
        response = "Invalid DirectMessageReplyModule configuration";
    } else {
        getIthValue(config.responses, response, iresponse);
    }

    // Add to response if condition is true
    auto _addToResponseIf = [&response](bool const condition, std::string const &str, char const *sep = " ") {
        addToResponseIf(condition, response, str, sep);
    };

    _addToResponseIf(!isDM,                      "Channel: " + std::string(channels.getName(channel)), "\n");
    _addToResponseIf( config.send_hops,          "Hop lim: " + std::to_string(mp.hop_limit) + "/" + std::to_string(mp.hop_start), "\n");
    _addToResponseIf(!config.send_hops,          "", "\n"); // Append newline if message isn't empty so far and not sending hops info
    _addToResponseIf(config.send_signal_metrics, "SNR: "     + toStringPrecision(1, mp.rx_snr)); // + " dB";
    _addToResponseIf(config.send_signal_metrics, "RSSI: "    + std::to_string(mp.rx_rssi));     // + " dBm";
    _addToResponseIf(config.echo_message, sourceShortName + ": " + message, "\n");

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
