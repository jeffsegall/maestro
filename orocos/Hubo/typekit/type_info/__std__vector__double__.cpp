/* Generated from orogen/templates/typekit/type_info/Info.cpp */

#include <Hubo/Types.hpp>
#include <Hubo/type_info/BoostSerialization.hpp>
#include <rtt/types/SequenceTypeInfo.hpp>



namespace orogen_typekits {
    struct std_vector__double_TypeInfo :
    
	public RTT::types::SequenceTypeInfo< ::std::vector< double > >
    
    {
        std_vector__double_TypeInfo()
	
            : RTT::types::SequenceTypeInfo< ::std::vector< double > >("/std/vector</double>") {}
	
    };

    RTT::types::TypeInfo* std_vector__double__TypeInfo()
    { return new std_vector__double_TypeInfo(); }
}

/* Generated from orogen/lib/orogen/templates/typekit/TemplateInstanciation.cpp */

#include <rtt/Port.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/Property.hpp>
#include <rtt/internal/DataSource.hpp>

template class RTT::OutputPort< ::std::vector< double > >;
template class RTT::InputPort< ::std::vector< double > >;
template class RTT::Property< ::std::vector< double > >;
template class RTT::Attribute< ::std::vector< double > >;

template class RTT::internal::DataSource< ::std::vector< double > >;
template class RTT::internal::ValueDataSource< ::std::vector< double > >;
template class RTT::internal::ConstantDataSource< ::std::vector< double > >;
template class RTT::internal::AssignableDataSource< ::std::vector< double > >;
template class RTT::internal::ReferenceDataSource< ::std::vector< double > >;



