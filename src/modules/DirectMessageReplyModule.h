#pragma once

#include "SinglePortModule.h"
#include "Singleton.h"
#include "mesh/generated/meshtastic/mesh.pb.h"

/**
 * A module that replies to direct messages with the info about the quality of
 * the signal and potentially a canned response.
 *
 * To configure the same canned response to all direct messages:
 *
 * config.queries = ""
 * config.responses = "canned reponse"
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
 */
class DirectMessageReplyModule : public SinglePortModule, public Singleton<DirectMessageReplyModule> {
public:
  using ConfigType = meshtastic_ModuleConfig_DirectMessageReplyConfig;

  static ConfigType getDefaultConfig();
  static void       setDefault();

  /// Constructor: name is for debugging output
  DirectMessageReplyModule() : SinglePortModule("directmessagereply", meshtastic_PortNum_TEXT_MESSAGE_APP) {
    LOG_DEBUG("DirectMessageReplyModule constructor");
  }

private:
  /// Called to handle a particular incoming message
  ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;

  void sendReply(const meshtastic_MeshPacket &mp, const std::string &response);
};
