/* Generated from orogen/lib/orogen/templates/typekit/typelib/Registration.hpp */

#ifndef __OROGEN_GENERATED_HUBO_TYPELIB_REGISTRATION_HPP
#define __OROGEN_GENERATED_HUBO_TYPELIB_REGISTRATION_HPP

namespace Typelib {
    class Registry;
}

namespace orogen_transports {
    class TypelibMarshallerBase;
}

namespace orogen_typekits {
    
    /** Creates and returns a TypeInfo object for HuboData */
    orogen_transports::TypelibMarshallerBase* HuboData_TypelibMarshaller(Typelib::Registry const& registry);
    
    /** Creates and returns a TypeInfo object for ::std::string */
    orogen_transports::TypelibMarshallerBase* std_string_TypelibMarshaller(Typelib::Registry const& registry);
    
    /** Creates and returns a TypeInfo object for ::std::vector< double > */
    orogen_transports::TypelibMarshallerBase* std_vector__double__TypelibMarshaller(Typelib::Registry const& registry);
    
}

#endif


