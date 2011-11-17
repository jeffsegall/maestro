/* Generated from orogen/lib/orogen/templates/typekit/typelib/TransportPlugin.hpp */

#ifndef __OROGEN_GENERATED_HUBO_TYPELIB_PLUGIN_HPP
#define __OROGEN_GENERATED_HUBO_TYPELIB_PLUGIN_HPP

#include <rtt/types/TransportPlugin.hpp>

namespace Typelib
{
    // Forward declaration for the plugin
    class Registry;
}

namespace orogen_typekits {
    class HuboTypelibTransportPlugin
        : public RTT::types::TransportPlugin
    {
        Typelib::Registry* m_registry;

    public:
        HuboTypelibTransportPlugin();
        ~HuboTypelibTransportPlugin();
        virtual bool registerTransport(std::string type_name, RTT::types::TypeInfo* ti);
        virtual std::string getTransportName() const;
        virtual std::string getTypekitName() const;
        virtual std::string getName() const;

        static std::string getTlbPath();
    };

    extern HuboTypelibTransportPlugin HuboTypelibTransport;
}

#endif

