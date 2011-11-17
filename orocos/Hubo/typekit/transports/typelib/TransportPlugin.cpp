/* Generated from orogen/lib/orogen/templates/typekit/typelib/TransportPlugin.cpp */

#include <typelib/pluginmanager.hh>

#include "Registration.hpp"
#include "TransportPlugin.hpp"
#include <rtt/typelib/TypelibMarshallerBase.hpp>

#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/Logger.hpp>
using namespace RTT;
#ifdef HAS_ROSLIB
#include <ros/package.h>
#endif

#define TYPEKIT_PACKAGE_NAME_aux0(target) #target
#define TYPEKIT_PACKAGE_NAME_aux(target) "Hubo-typekit-" TYPEKIT_PACKAGE_NAME_aux0(target)
#define TYPEKIT_PACKAGE_NAME TYPEKIT_PACKAGE_NAME_aux(OROCOS_TARGET)

std::string orogen_typekits::HuboTypelibTransportPlugin::getTlbPath()
{
#ifdef HAS_ROSLIB
    using namespace ros::package;
    try {
        bool all_good = true, found = false;
        std::string ppath = getPath( "Hubo" );
        if ( !ppath.empty() ) {
            return ppath + "/typekit" + "/Hubo.tlb";
        } else
            log(Error) << "Not a ros package: " << "Hubo" << endlog();
    } catch(...) {
        log(Error) << "Not a ros package: " << "Hubo" << endlog();
    }
#endif
    return TYPEKIT_REGISTRY;
}

orogen_typekits::HuboTypelibTransportPlugin::HuboTypelibTransportPlugin()
    : m_registry(0)
{
    std::string path = getTlbPath();
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
        log(Error) << "the Typelib transport will not be available for types defined in this typekit" << endlog();
    }
}

orogen_typekits::HuboTypelibTransportPlugin::~HuboTypelibTransportPlugin()
{
    delete m_registry;
}

bool orogen_typekits::HuboTypelibTransportPlugin::registerTransport(std::string type_name, RTT::types::TypeInfo* ti)
{
    if (!m_registry)
        return false;

    
    if ("/HuboData" == type_name)
    {
        try {
            ti->addProtocol(orogen_transports::TYPELIB_MARSHALLER_ID,
                HuboData_TypelibMarshaller(*m_registry));
        }
        catch(std::runtime_error const& e)
        {
            log(Error) << "cannot register a typelib transport for " << type_name << endlog();
            log(Error) << "  the following error occured:" << endlog();
            log(Error) << "  " << e.what() << endlog();
            log(Error) << "  the registry can be found at " << TYPEKIT_REGISTRY << endlog();
            return false;
        }
        return true;
    }
    
    else if ("/std/string" == type_name)
    {
        try {
            ti->addProtocol(orogen_transports::TYPELIB_MARSHALLER_ID,
                std_string_TypelibMarshaller(*m_registry));
        }
        catch(std::runtime_error const& e)
        {
            log(Error) << "cannot register a typelib transport for " << type_name << endlog();
            log(Error) << "  the following error occured:" << endlog();
            log(Error) << "  " << e.what() << endlog();
            log(Error) << "  the registry can be found at " << TYPEKIT_REGISTRY << endlog();
            return false;
        }
        return true;
    }
    
    else if ("/std/vector</double>" == type_name)
    {
        try {
            ti->addProtocol(orogen_transports::TYPELIB_MARSHALLER_ID,
                std_vector__double__TypelibMarshaller(*m_registry));
        }
        catch(std::runtime_error const& e)
        {
            log(Error) << "cannot register a typelib transport for " << type_name << endlog();
            log(Error) << "  the following error occured:" << endlog();
            log(Error) << "  " << e.what() << endlog();
            log(Error) << "  the registry can be found at " << TYPEKIT_REGISTRY << endlog();
            return false;
        }
        return true;
    }
    
    return false;
}
std::string orogen_typekits::HuboTypelibTransportPlugin::getTransportName() const
{ return "Typelib"; }
std::string orogen_typekits::HuboTypelibTransportPlugin::getTypekitName() const
{ return "/orogen/Hubo"; }
std::string orogen_typekits::HuboTypelibTransportPlugin::getName() const
{ return "/orogen/Hubo/TYPELIB"; }

ORO_TYPEKIT_PLUGIN(orogen_typekits::HuboTypelibTransportPlugin);

