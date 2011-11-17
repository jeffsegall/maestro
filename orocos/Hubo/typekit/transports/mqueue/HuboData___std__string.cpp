/* Generated from orogen/lib/orogen/templates/typekit/mqueue/Type.cpp */

#include "Types.hpp"
#include "transports/mqueue/Registration.hpp"



#include "transports/typelib/Registration.hpp"
#include <rtt/typelib/MQTypelibMarshaller.hpp>


namespace Typelib
{
    class Registry;
}

namespace orogen_typekits {
    RTT::types::TypeMarshaller*  HuboData_MQueueTransport(Typelib::Registry const& registry)
    {
        
        orogen_transports::TypelibMarshallerBase* marshaller =
            HuboData_TypelibMarshaller(registry);

        return new orogen_transports::MQTypelibMarshaller< HuboData >(marshaller);
        
    }
}


/* Generated from orogen/lib/orogen/templates/typekit/mqueue/Type.cpp */

#include "Types.hpp"
#include "transports/mqueue/Registration.hpp"



#include "transports/typelib/Registration.hpp"
#include <rtt/typelib/MQTypelibMarshaller.hpp>


namespace Typelib
{
    class Registry;
}

namespace orogen_typekits {
    RTT::types::TypeMarshaller*  std_string_MQueueTransport(Typelib::Registry const& registry)
    {
        
        orogen_transports::TypelibMarshallerBase* marshaller =
            std_string_TypelibMarshaller(registry);

        return new orogen_transports::MQTypelibMarshaller< ::std::string >(marshaller);
        
    }
}

