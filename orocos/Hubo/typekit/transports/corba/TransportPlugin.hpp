/* Generated from orogen/lib/orogen/templates/typekit/corba/TransportPlugin.hpp */

#ifndef __OROGEN_GENERATED_HUBO_CORBA_PLUGIN_HPP
#define __OROGEN_GENERATED_HUBO_CORBA_PLUGIN_HPP

#include <rtt/types/TransportPlugin.hpp>

namespace orogen_typekits {
    class HuboCorbaTransportPlugin
        : public RTT::types::TransportPlugin
    {
    public:
        virtual bool registerTransport(std::string type_name, RTT::types::TypeInfo* ti);
        virtual std::string getTransportName() const;
        virtual std::string getTypekitName() const;
        virtual std::string getName() const;
    };

    extern HuboCorbaTransportPlugin HuboCorbaTransport;
}

#endif

