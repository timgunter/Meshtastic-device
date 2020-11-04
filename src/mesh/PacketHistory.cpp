#include "PacketHistory.h"
#include "configuration.h"
#include "mesh-pb-constants.h"

PacketHistory::PacketHistory()
{
    recentPackets.reserve(MAX_NUM_NODES); // Prealloc the worst case # of records - to prevent heap fragmentation
                                          // setup our periodic task
}

/**
 * Update recentBroadcasts and return true if we have already seen this packet
 */
bool PacketHistory::wasSeenRecently(const MeshPacket *p, bool withUpdate)
{
    if (p->id == 0) {
        DEBUG_MSG("Ignoring message with zero id\n");
        return false; // Not a floodable message ID, so we don't care
    }

    uint32_t now = millis();

    for(auto iter = recentPackets.begin(); iter != recentPackets.end();) {
        // In this case, the const_cast is ok because we will only modify the "rxTimeMsec" member
        // which doesn't change the hash for this element.
        PacketRecord &r = const_cast<PacketRecord &>(*iter);

        // This assert statement can be used to confirm that we can safely modify "rxTimeMsec"
        using PRHash = std::hash<PacketRecord>;
        assert(PRHash()(r) == PRHash()(PacketRecord(r.sender, r.id, now)));

        if ((now - r.rxTimeMsec) >= FLOOD_EXPIRE_TIME) {
            // DEBUG_MSG("Deleting old broadcast record %d\n", i);
            iter = recentPackets.erase(iter); // delete old record
        } else {
            if (r.id == p->id && r.sender == p->from) {
                DEBUG_MSG("Found existing packet record for fr=0x%x,to=0x%x,id=%d\n", p->from, p->to, p->id);

                // Update the time on this record to now
                if (withUpdate)
                    r.rxTimeMsec = now;
                return true;
            }

            ++iter;
        }
    }

    // Didn't find an existing record, make one
    if (withUpdate) {
        recentPackets.emplace(p->from, p->id, now);
        printPacket("Adding packet record", p);
    }

    return false;
}
