#pragma once

#include "Router.h"
#include <queue>
#include <unordered_set>

#include "hash_combine.h"

using namespace std;

/// We clear our old flood record five minute after we see the last of it
#define FLOOD_EXPIRE_TIME (5 * 60 * 1000L)

/**
 * A record of a recent message broadcast
 */
struct PacketRecord {
    NodeNum sender = 0;
    PacketId id = 0;
    uint32_t rxTimeMsec = 0; // Unix time in msecs - the time we received it

    PacketRecord(NodeNum const &sender_, PacketId const &id_, uint32_t const &rxTimeMsec_)
    : sender(sender_)
    , id(id_)
    , rxTimeMsec(rxTimeMsec_)
    {}

    bool operator==(const PacketRecord &p) const { return sender == p.sender && id == p.id; }
};

// std::hash<PacketRecord> template specialization
namespace std
{
    template<>
    struct hash<PacketRecord>
    {
        size_t operator()(PacketRecord const &p) const
        {
            return make_hash(p.sender, p.id);
        }
    };
} // namespace std

/// Order packet records by arrival time, we want the oldest packets to be in the front of our heap
class PacketRecordOrderFunction
{
  public:
    size_t operator()(const PacketRecord &p1, const PacketRecord &p2) const
    {
        // If the timer ticks have rolled over the difference between times will be _enormous_.  Handle that case specially
        uint32_t t1 = p1.rxTimeMsec, t2 = p2.rxTimeMsec;

        if (t1 - t2 > UINT32_MAX / 2) {
            // time must have rolled over, swap them because the new little number is 'bigger' than the old big number
            t1 = t2;
            t2 = p1.rxTimeMsec;
        }

        return t1 > t2;
    }
};

/**
 * This is a mixin that adds a record of past packets we have seen
 */
class PacketHistory
{
  private:
    /** FIXME: really should be a std::unordered_set with the key being sender,id.
     * This would make checking packets in wasSeenRecently faster.
     */
    unordered_set<PacketRecord> recentPackets;

  public:
    PacketHistory();

    /**
     * Update recentBroadcasts and return true if we have already seen this packet
     *
     * @param withUpdate if true and not found we add an entry to recentPackets
     */
    bool wasSeenRecently(const MeshPacket *p, bool withUpdate = true);
};
