#pragma once

#include <set>

#include "MultiPortModule.h"
#include "Singleton.h"

/// This module replies to position updates with a configurable message about
/// the positions of and distance and bearing between the sender and receiver.
/// It can be used purely to provide this information or it can also be used to
/// setup a game consisting of a course of nodes, somewhat like geocaching an
/// orienteering course. In this mode, as participants send position updates
/// from within a certain distance of this node, they unlock clues about the
/// next node in the sequence which will be received via direct message.

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
};
