#include "DirectMessageReplyModule.h"

#include <cassert>
#include <cstring>

#include <string>
#include <iomanip>
#include <sstream>

#include <algorithm>

#include "Meshservice.h"
//#include "configuration.h"
//#include "main.h"

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
