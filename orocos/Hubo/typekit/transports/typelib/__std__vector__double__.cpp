/* Generated from orogen/lib/orogen/templates/typekit/typelib/Type.cpp */

#include "Types.hpp"
#include <rtt/typelib/TypelibMarshaller.hpp>
#include "transports/typelib/Registration.hpp"

orogen_transports::TypelibMarshallerBase* orogen_typekits::std_vector__double__TypelibMarshaller(Typelib::Registry const& registry)
{
    return new orogen_transports::TypelibMarshaller< ::std::vector< double > >("/std/vector</double>", registry);
}



