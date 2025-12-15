#pragma once

#include <set>

#include "MultiPortModule.h"
#include "Singleton.h"

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

        static ConfigType getDefaultConfig();
        static void       setDefault();

        PositionUpdateReplyModule();

    protected:
        /// Called to handle a particular incoming message
        ProcessMessage handleReceived(const meshtastic_MeshPacket &mp) override;

    private:
        ProcessMessage handleReceivedTextMessage(const meshtastic_MeshPacket &mp);
        ProcessMessage handleReceivedPosition(   const meshtastic_MeshPacket &mp);

        void sendReply(const meshtastic_MeshPacket &mp, const std::string &response);

        std::set<NodeNum> m_monitored;

        static bool hasNextNode(ConfigType const &config);
};
