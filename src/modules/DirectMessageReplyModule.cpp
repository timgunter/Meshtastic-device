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

#ifndef LOG_PREFIX_DMR
/// Make messages cyan and add module specific prefix
//#   define LOG_PREFIX_DMR "\u001b[36m" "[DirMsgRep] " // Doesn't work. maybe due to use of vsnprintf in the logger?
#   define LOG_PREFIX_DMR "[DirMsgRep] "
#endif

#ifndef LOG_PREFIX
#   define LOG_PREFIX LOG_PREFIX_DMR
#endif

#include "LogUtils.h"


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

ProcessMessage DirectMessageReplyModule::handleReceived(meshtastic_MeshPacket const &mp) {
    auto const &config = getConfig();

    if (!config.enabled) {
        LOG_DEBUG_PFX("DirectMessageReplyModule is disabled, ignoring message");
        return ProcessMessage::CONTINUE;
    }

    auto        const &channel         = mp.channel;
    auto        const &p               = mp.decoded;
    auto        const  source          = getSource( mp);
    std::string const  message         = getMessage(mp);
    auto        const &dest            = (p.dest ? p.dest : mp.to);
    auto        const &myID            = nodeDB->getNodeNum();
    bool        const  isDM            = (dest != 0);
    auto        const  sourceShortName = getNodeShortName(source);

    if (p.reply_id != 0) {
        LOG_DEBUG_PFX("Skipping reply to message ID %u from %u", p.reply_id, source);
        return ProcessMessage::CONTINUE;
    }

    if (dest != myID) {
        LOG_DEBUG_PFX("Ignoring message not for us(%u) decoded dest: %u packet dest: %u", myID, p.dest, mp.to);
        return ProcessMessage::CONTINUE;
    }

    // See if message matches as query and provide appropriate response if so.
    auto const numQueries   = getNumValues(config.queries);
    auto const numResponses = getNumValues(config.responses);

    if(numQueries > 1 && numQueries != numResponses) {
        LOG_WARN_PFX("DirectMessageReplyModule: number of queries (%lu) is > 1 and does not match number of responses (%lu)", numQueries, numResponses);
    }

    size_t      iresponse = 0;
    std::string response;

    // If there are no queries, or only one query and it matches the message
    if(numQueries == 0 || (numQueries == 1 && equalIgnoreCase(message, config.queries))) {
        // 0 or 1 queries and multiple responses; choose response based on (source % numResponses)
        getIthValueModNum(config.responses, response, iresponse, numResponses);
        LOG_DEBUG_PFX("Lookup response by (source %% numResp): (%u, %lu) nqueries: %lu response[%lu]: %s", source, numResponses, numQueries, iresponse, response.c_str());
    } else if(numQueries > 1) {
        // If more than 1 query, see if any match the message, and use corresponding response
        if(findMatch(config.queries, message, iresponse)) {
            LOG_DEBUG_PFX("matched query index %lu", iresponse);
        } else { // Otherwise, use the first response
            iresponse = 0;
            LOG_DEBUG_PFX("no match found, using response 0");
        }
        assert(iresponse < numResponses);
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
    meshtastic_MeshPacket const &mp
    , std::string         const &response
) {
    auto const source  = getSource(mp);
    auto const channel = getConfig().channel;
    LOG_DEBUG_PFX("Replying to %u with: %s", source, response.c_str());
    reply_utils::sendReply(mp, allocDataPacket(), response, channel);
}
