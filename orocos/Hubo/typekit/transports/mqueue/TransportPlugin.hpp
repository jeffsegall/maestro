/* Generated from orogen/lib/orogen/templates/typekit/mqueue/TransportPlugin.hpp */

#ifndef __OROGEN_GENERATED_HUBO_MQUEUE_PLUGIN_HPP
#define __OROGEN_GENERATED_HUBO_MQUEUE_PLUGIN_HPP

#include <rtt/types/TransportPlugin.hpp>

namespace Typelib {
    class Registry;
}
namespace orogen_typekits {
    class HuboMQueueTransportPlugin
        : public RTT::types::TransportPlugin
    {
        Typelib::Registry* m_registry;

    public:
        HuboMQueueTransportPlugin();
        ~HuboMQueueTransportPlugin();

        virtual bool registerTransport(std::string type_name, RTT::types::TypeInfo* ti);
        virtual std::string getTransportName() const;
        virtual std::string getTypekitName() const;
        virtual std::string getName() const;
    };

    extern HuboMQueueTransportPlugin HuboMQueueTransport;
}

#endif

