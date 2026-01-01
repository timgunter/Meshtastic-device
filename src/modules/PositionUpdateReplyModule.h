#pragma once

#include <map>
#include <string>

#include "MultiPortModule.h"
#include "Singleton.h"
#include "GeoCoord.h"
#include "ReplyUtils.h"

/// This module replies to position updates with a configurable message about
/// the positions of and distance and bearing between the sender and receiver.
/// It can be used purely to provide this information or it can also be used to
/// setup a game consisting of a course of nodes, somewhat like geocaching or an
/// orienteering course. In this mode, as participants send position updates
/// from within a certain distance of this node, they unlock clues about the
/// next node in the sequence which will be received via direct message.
///
/// When a participant sends a position update from within a preconfigured distance
/// of the node, the node can respond with the name of the next node in the course
/// and the "code word" to send via direct message to the node to begin receiving
/// responses from it.
///
/// Each node in the course can be configure with a single next node/code word pair
/// or multiple. Similar to the "DirectMessageReplyModule", this is done by providing
/// a '|' delimited list of next nodes in the config.next_node field and a '|' delimited
/// list of code words in the config.next_code_word field.
///
/// If more than one pair is set, each node will use (source_address % num_next_nodes)
/// and (source_address % num_next_code_words) to determine which values are sent
/// in the reply. In general, it is expected that num_next_nodes == num_next_code_words,
/// but there may be cases in which leaving one of the lists empty is desirable. Or
/// perhaps even having different non-zero length lists.

class PositionUpdateReplyModule : public MultiPortModule, public Singleton<PositionUpdateReplyModule> {
    public:
        using ConfigType = meshtastic_ModuleConfig_PositionUpdateReplyConfig;

        static ConfigType  getDefaultConfig();
        static ConfigType &getConfig( ) { return moduleConfig.position_update_reply; }
        static void        setDefault() { getConfig() = getDefaultConfig(); }

        PositionUpdateReplyModule();

    protected:
        /// Called to handle a particular incoming message
        ProcessMessage handleReceived(meshtastic_MeshPacket const &mp) override;

    private:
        ProcessMessage handleReceivedTextMessage(meshtastic_MeshPacket const &mp);
        ProcessMessage handleReceivedPosition(   meshtastic_MeshPacket const &mp);

        /// Reply to sender of mp. Fills out destination, copies response and sends
        void sendReply(meshtastic_MeshPacket const &mp, std::string const &response);

        /// Default codeword
        static char const constexpr *s_defaultCodeWord = "start";
        /// Return true of string matches default codeword
        static bool isDefaultCodeWord(std::string const &str);

        /// Returns true if configured to provide a clue to a subsquent node
        static bool   hasNextNode();

        /// Return number of codewords for this node
        static size_t getNumCodeWords();

        /// Retrieve current position to report against for source
        GeoCoord getLocalGeoCoord(uint32_t const source) const;

        /// Check if message matches codeword
        bool matchesCodeWord(std::string const &message, size_t &icodeWord) const;

        /// Retrieve current code word by index
        bool getCodeWord(size_t const index, std::string &codeWord) const;

        /// Retrieve next node and next code word in sequence
        void getNextNodeCode(uint32_t const source, std::string &nextNode, std::string &nextCodeWord) const;

        /// Return number of nodes being monitored
        size_t numMonitored() const;

        /// Return true if source position updates are monitored
        bool isMonitored(uint32_t const source) const;

        /// Set source index
        void setSourceIndex(  uint32_t const source, size_t const index);

        /// Unset source index
        void unsetSourceIndex(uint32_t const source);

        /// Return true if source is being monitored and retrieve current source index
        bool getSourceIndex(uint32_t const source, size_t &index) const;

        std::map<NodeNum, size_t> m_monitored;
};

/// Return true of string matches default codeword
inline bool PositionUpdateReplyModule::isDefaultCodeWord(std::string const &str) {
    return reply_utils::equalIgnoreCase(str, s_defaultCodeWord);
}

/// Return number of codewords for this node
inline size_t PositionUpdateReplyModule::getNumCodeWords() {
    return reply_utils::getNumValues(getConfig().start_code_word);
}

/// Retrieve current code word by index
inline bool PositionUpdateReplyModule::getCodeWord(size_t const index, std::string &codeWord) const {
    return reply_utils::getIthValue(getConfig().start_code_word, codeWord, index);
}

/// Return number of nodes being monitored
inline size_t PositionUpdateReplyModule::numMonitored() const {
    return m_monitored.size();
}

/// Return true if source position updates are monitored
inline bool PositionUpdateReplyModule::isMonitored(uint32_t const source) const {
    return m_monitored.count(source) > 0;
}

/// Set source index
inline void PositionUpdateReplyModule::setSourceIndex(  uint32_t const source, size_t const index) {
    m_monitored[source] = index;
}

/// Unset source index
inline void PositionUpdateReplyModule::unsetSourceIndex(uint32_t const source) {
    m_monitored.erase(source);
}
