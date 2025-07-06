#pragma once
#include <cassert>

#include <set>

#include "MeshModule.h"
#include "Router.h"

/**
 * Simple module for receiving or sending on more than one port.
 */
class MultiPortModule : public MeshModule
{
  protected:
    using PortSet = std::set<meshtastic_PortNum>;

    PortSet ourPorts;

  public:
    /** Constructor
     * name is for debugging output
     */
    template<typename... Args>
    MultiPortModule(const char *_name, Args &&... ports) : MeshModule(_name), ourPorts(std::forward<Args>(ports)...)
    {
        assert(ourPorts.size() > 0);
    }

    virtual ~MultiPortModule() = default;

  protected:
    /**
     * @return true if you want to receive the specified portnum
     */
    virtual bool wantPacket(const meshtastic_MeshPacket *p) override { return ourPorts.count(p->decoded.portnum) == 1; }

    /**
     * Return a mesh packet which has been preinited as a data packet with a particular port number.
     * You can then send this packet (after customizing any of the payload fields you might need) with
     * service->sendToMesh()
     */
    meshtastic_MeshPacket *allocDataPacket()
    {
        if(ourPorts.empty()) return nullptr;

        return allocDataPacket(*ourPorts.begin());
    }

    meshtastic_MeshPacket *allocDataPacket(meshtastic_PortNum const port)
    {
        // Update our local node info with our position (even if we don't decide to update anyone else)
        meshtastic_MeshPacket *p = router->allocForSending();
        p->decoded.portnum = port;

        return p;
    }
};
