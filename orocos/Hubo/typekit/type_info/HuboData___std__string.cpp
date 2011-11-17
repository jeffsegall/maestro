/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <Hubo/Types.hpp>
#include <Hubo/type_info/BoostSerialization.hpp>
#include <rtt/types/StructTypeInfo.hpp>



namespace orogen_typekits {
    struct HuboDataTypeInfo :
    
	public RTT::types::StructTypeInfo< HuboData >
    
    {
        HuboDataTypeInfo()
	
            : RTT::types::StructTypeInfo< HuboData >("/HuboData") {}
	
    };

    RTT::types::TypeInfo* HuboData_TypeInfo()
    { return new HuboDataTypeInfo(); }
}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::OutputPort< HuboData >;
template class RTT::InputPort< HuboData >;
template class RTT::Property< HuboData >;
template class RTT::Attribute< HuboData >;

template class RTT::internal::DataSource< HuboData >;
template class RTT::internal::ValueDataSource< HuboData >;
template class RTT::internal::ConstantDataSource< HuboData >;
template class RTT::internal::AssignableDataSource< HuboData >;
template class RTT::internal::ReferenceDataSource< HuboData >;




/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <Hubo/Types.hpp>
#include <Hubo/type_info/BoostSerialization.hpp>
#include <rtt/types/SequenceTypeInfo.hpp>

#include <rtt/typekit/StdStringTypeInfo.hpp>



namespace orogen_typekits {
    struct std_stringTypeInfo :
    
        public RTT::types::StdStringTypeInfo
    
    {
        std_stringTypeInfo()
	
	    : RTT::types::StdStringTypeInfo("/std/string") {}
        
    };

    RTT::types::TypeInfo* std_string_TypeInfo()
    { return new std_stringTypeInfo(); }
}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::OutputPort< ::std::string >;
template class RTT::InputPort< ::std::string >;
template class RTT::Property< ::std::string >;
template class RTT::Attribute< ::std::string >;

template class RTT::internal::DataSource< ::std::string >;
template class RTT::internal::ValueDataSource< ::std::string >;
template class RTT::internal::ConstantDataSource< ::std::string >;
template class RTT::internal::AssignableDataSource< ::std::string >;
template class RTT::internal::ReferenceDataSource< ::std::string >;



