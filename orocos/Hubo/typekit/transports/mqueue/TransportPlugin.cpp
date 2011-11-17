/* Generated from orogen/lib/orogen/templates/typekit/mqueue/TransportPlugin.cpp */

// First load all RTT interfaces so that we get all "extern template"
// declarations in the TypekitImpl files
#include "transports/mqueue/Registration.hpp"
#include "transports/mqueue/TransportPlugin.hpp"
#include <rtt/transports/mqueue/MQLib.hpp>
#include <rtt/types/TypekitPlugin.hpp>
using namespace RTT;

#include "transports/typelib/TransportPlugin.hpp"
#include <typelib/pluginmanager.hh>
#include <typelib/registry.hh>
#include <rtt/Logger.hpp>

orogen_typekits::HuboMQueueTransportPlugin::HuboMQueueTransportPlugin()
    : m_registry(0)
{
    std::string path = HuboTypelibTransportPlugin::getTlbPath();
    try
    {
        m_registry = Typelib::PluginManager::load("tlb", path);
    }
    catch(std::exception const& e) {
        log(Error) << "cannot load the typekit's Typelib registry from" << endlog();
        log(Error) << "  " << path << endlog();
#ifndef HAS_ROSLIB
        log(Error) << "remember to do 'make install' before you use the oroGen-generated libraries ?" << endlog();
#endif
        log(Error) << endlog();
        log(Error) << "the MQueue transport will not be available for types defined in this typekit" << endlog();
    }
}

orogen_typekits::HuboMQueueTransportPlugin::~HuboMQueueTransportPlugin()
{
    delete m_registry;
}

bool orogen_typekits::HuboMQueueTransportPlugin::registerTransport(std::string type_name, RTT::types::TypeInfo* ti)
{
    if (!m_registry)
        return false;

    
    if ("/HuboData" == type_name)
    {
        ti->addProtocol(ORO_MQUEUE_PROTOCOL_ID,
            HuboData_MQueueTransport(*m_registry));
        return true;
    }
    
    else if ("/std/string" == type_name)
    {
        ti->addProtocol(ORO_MQUEUE_PROTOCOL_ID,
            std_string_MQueueTransport(*m_registry));
        return true;
    }
    
    else if ("/std/vector</double>" == type_name)
    {
        ti->addProtocol(ORO_MQUEUE_PROTOCOL_ID,
            std_vector__double__MQueueTransport(*m_registry));
        return true;
    }
    
    return false;
}
std::string orogen_typekits::HuboMQueueTransportPlugin::getTransportName() const
{ return "MQueue"; }
std::string orogen_typekits::HuboMQueueTransportPlugin::getTypekitName() const
{ return "/orogen/Hubo"; }
std::string orogen_typekits::HuboMQueueTransportPlugin::getName() const
{ return "/orogen/Hubo/MQueue"; }

ORO_TYPEKIT_PLUGIN(orogen_typekits::HuboMQueueTransportPlugin);

