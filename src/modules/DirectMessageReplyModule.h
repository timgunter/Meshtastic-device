#pragma once

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

/// Do these belong in a better more general location?
namespace reply_utils {
    /// Convert float to string with specified precision
    std::string toStringPrecision(int const precision, float const value);

    /// Copy a string into the payload, ensuring it does not exceed the payload size
    void copyStringToPayload(meshtastic_Data_payload_t &payload, std::string const &str);

    /// Return true if channel is private
    bool isPrivateChannel(ChannelIndex const channel);

    /// Retrieve short/long name for node
    std::string getNodeShortName(uint32_t const nodeNum = nodeDB->getNodeNum());
    std::string getNodeLongName( uint32_t const nodeNum = nodeDB->getNodeNum());

    /// Returns true if strings match ignoring case
    bool equalIgnoreCase(std::string const &left, std::string const &right);

    /// Get substring using start and end positions(instead of start and length)
    std::string startToEnd(std::string const &str, std::string::size_type const start, std::string::size_type const end);

    /// Returns true if pos represents end of string (npos)
    bool atEnd(std::string::size_type const pos);

    /// Set start to current end, and increment past delimiter
    void setStart(std::string::size_type &start, std::string::size_type const end);

    /// Search a delimited string of values for a particular value. "i" is 0-based index to value if found
    bool findMatch(std::string const &values, std::string const &value, size_t &I, char const delim = '|');

    /// Count number of values in a delimited string
    size_t getNumValues(std::string const &values, char const delim = '|');

    /// Retrieve Ith(0-based) value from a delimited list of values
    bool getIthValue(std::string const &values, std::string &value, size_t const I, char const delim = '|');

    /// Append a string to a response string with a separator
    void addToResponse(std::string &response, std::string const &str, char const *sep = " ");

    /// Append a string to a response string if a condition is true
    void addToResponseIf(bool const condition, std::string &response, std::string const &str, char const *sep = " ");
}
