#pragma once

#include <string>

#pragma push_macro("abs")
#pragma push_macro("round")
#undef abs
#undef round
#include <chrono> // Include here so that <cmath>'s round() macro doesn't conflict with std::chrono::round
#pragma pop_macro( "abs")
#pragma pop_macro( "round")

#include "SinglePortModule.h"
#include "Singleton.h"
#include "mesh/generated/meshtastic/mesh.pb.h"

/**
 * A module that replies to direct messages with the info about the quality of
 * the signal and potentially a canned response.
 * This module automatically replies to direct messages with pre-configured responses.
 * These responses can be configured so that different responses will be provided based
 * upon different "queries" in the received direct message.
 * Other options allow the response to contain other info like signal metrics, hop info, etc.
 *
 * To configure the same canned response to all direct messages:
 *
 * config.queries = ""
 * config.responses = "canned response"
 *
 * To setup multiple different responses to different incoming queries, queries
 * and responses are delimited with '|' characters in the config.
 *
 * For example if configured like this:
 *
 * config.queries = "1|2|3|4|5"
 * config.responses = "one|two|three|four|five"
 *
 * A received direct message of "3" will receive a reply message containing "three".
 *
 * If the received message doesn't match any of the queries, the first response
 * will be used. To have this generic response not correspond to any query,
 * make the first query empty by making the first character of "queries" a '|'.
 *
 * config.queries = "|1|2|3|4|5"
 * config.responses = "default response|one|two|three|four|five"
 *
 * In this configuration, a received direct message that doesn't correspond to
 * any of the queries will receive a reply of "default response", and again a
 * query of "3" will recieve a reply containing "three".
 *
 * If there are 0 or 1 queries and more than one response set, a response will
 * be selected by using (source_address & nresponses). The full set of ways the
 * queries and responses can be configured is as follows.
 *
 * nqueries == 0 && nresponses == 0        => no custom message will be added to the response
 * nqueries == 0 && nresponses == 1        => always use the single response
 * nqueries == 1 && nresponses == 1        => if message matches query, use the single response
 * nqueries == 0 && nresponses >  1        => use (source_address % nresponses) to select response
 * nqueries == 1 && nresponses >  1        => if message matches query, use (source_address % nresponses)th response
 * nqueries >  1 && nresponses == nqueries => if message matches nth query, use nth response
 * nqueries >  1 && nresponses >  nqueries => same as above, but extra responses are ignored
 */
class DirectMessageReplyModule : public SinglePortModule, public Singleton<DirectMessageReplyModule> {
public:
  using ConfigType = meshtastic_ModuleConfig_DirectMessageReplyConfig;

  static ConfigType  getDefaultConfig();
  static ConfigType &getConfig( ) { return moduleConfig.direct_message_reply; }
  static void        setDefault() { getConfig() = getDefaultConfig(); }

  /// Constructor: name is for debugging output
  DirectMessageReplyModule() : SinglePortModule("directmessagereply", meshtastic_PortNum_TEXT_MESSAGE_APP) {
    LOG_DEBUG("DirectMessageReplyModule constructor");
  }

private:
  /// Called to handle a particular incoming message
  ProcessMessage handleReceived(meshtastic_MeshPacket const &mp) override;

  /// Reply to sender of mp. Fills out destination, copies response and sends
  void sendReply(meshtastic_MeshPacket const &mp, std::string const &response);
};
