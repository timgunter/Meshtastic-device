#pragma once

#include <string>

#include "mesh/generated/meshtastic/mesh.pb.h"

#include "Channels.h"

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

    /// Retrieve Ith(0-based) value modulo N from a delimited list of values(if N == 0, getNumValues(values) will be used)
    bool getIthValueModNum(std::string const &values, std::string &value, size_t const I, size_t N = 0, char const delim = '|');

    /// Append a string to a response string with a separator
    void addToResponse(std::string &response, std::string const &str, char const *sep = " ");

    /// Append a string to a response string if a condition is true
    void addToResponseIf(bool const condition, std::string &response, std::string const &str, char const *sep = " ");

    /// Get source from packet
    uint32_t getSource(meshtastic_MeshPacket const &mp);

    /// Get message from packet
    std::string getMessage(meshtastic_MeshPacket const &mp);

    /// Reply to sender of mp. Fills out destination, copies response and sends
    void sendReply(
        const   meshtastic_MeshPacket &mp
        , meshtastic_MeshPacket       *reply
        , const std::string           &response
        , const uint32_t               channel   = 0
        , const size_t                 maxLength = 226
    );

    /// Inline functions

    /// Returns true if pos represents end of string (npos)
    inline bool atEnd(std::string::size_type const pos) { return pos == std::string::npos; }

    /// Get source from packet
    inline uint32_t getSource(meshtastic_MeshPacket const &mp) {
        return (mp.decoded.source ? mp.decoded.source : mp.from); // Does this always come from mp?
    }

    /// Get message from packet
    inline std::string getMessage(meshtastic_MeshPacket const &mp) {
        return std::string{reinterpret_cast<const char *>(mp.decoded.payload.bytes), mp.decoded.payload.size};
    }
}
