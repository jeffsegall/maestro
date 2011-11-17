/* Generated from orogen/lib/orogen/templates/typekit/corba/TransportPlugin.cpp */

// First load all RTT interfaces so that we get all "extern template"
// declarations in the TypekitImpl files
#include "transports/corba/Registration.hpp"
#include "transports/corba/TransportPlugin.hpp"
#include <rtt/transports/corba/CorbaLib.hpp>
#include <rtt/types/TypekitPlugin.hpp>
using namespace RTT;

bool orogen_typekits::HuboCorbaTransportPlugin::registerTransport(std::string type_name, RTT::types::TypeInfo* ti)
{
    
    if ("/HuboData" == type_name)
    {
        ti->addProtocol(ORO_CORBA_PROTOCOL_ID,
            HuboData_CorbaTransport());
        return true;
    }
    
    else if ("/std/string" == type_name)
    {
        ti->addProtocol(ORO_CORBA_PROTOCOL_ID,
            std_string_CorbaTransport());
        return true;
    }
    
    else if ("/std/vector</double>" == type_name)
    {
        ti->addProtocol(ORO_CORBA_PROTOCOL_ID,
            std_vector__double__CorbaTransport());
        return true;
    }
    
    return false;
}
std::string orogen_typekits::HuboCorbaTransportPlugin::getTransportName() const
{ return "CORBA"; }
std::string orogen_typekits::HuboCorbaTransportPlugin::getTypekitName() const
{ return "/orogen/Hubo"; }
std::string orogen_typekits::HuboCorbaTransportPlugin::getName() const
{ return "/orogen/Hubo/CORBA"; }

ORO_TYPEKIT_PLUGIN(orogen_typekits::HuboCorbaTransportPlugin);

