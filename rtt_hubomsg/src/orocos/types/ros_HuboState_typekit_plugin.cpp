#include <hubomsg/boost/HuboState.h>
#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/StructTypeInfo.hpp>
#include <rtt/types/PrimitiveSequenceTypeInfo.hpp>
#include <rtt/types/CArrayTypeInfo.hpp>
#include <vector>

// Note: we need to put these up-front or we get gcc compiler warnings:
// <<warning: type attributes ignored after type is already defined>>        
template class RTT_EXPORT RTT::internal::DataSourceTypeInfo< hubomsg::HuboState >;
template class RTT_EXPORT RTT::internal::DataSource< hubomsg::HuboState >;
template class RTT_EXPORT RTT::internal::AssignableDataSource< hubomsg::HuboState >;
template class RTT_EXPORT RTT::internal::AssignCommand< hubomsg::HuboState >;
template class RTT_EXPORT RTT::internal::ValueDataSource< hubomsg::HuboState >;
template class RTT_EXPORT RTT::internal::ConstantDataSource< hubomsg::HuboState >;
template class RTT_EXPORT RTT::internal::ReferenceDataSource< hubomsg::HuboState >;
template class RTT_EXPORT RTT::OutputPort< hubomsg::HuboState >;
template class RTT_EXPORT RTT::InputPort< hubomsg::HuboState >;
template class RTT_EXPORT RTT::Property< hubomsg::HuboState >;
template class RTT_EXPORT RTT::Attribute< hubomsg::HuboState >;
template class RTT_EXPORT RTT::Constant< hubomsg::HuboState >;

namespace ros_integration {
  using namespace RTT;
    // Factory function
    
        void rtt_ros_addType_hubomsg_HuboState() {
             // Only the .msg type is sent over ports. The msg[] (variable size) and  cmsg[] (fixed size) exist only as members of larger messages
             RTT::types::Types()->addType( new types::StructTypeInfo<hubomsg::HuboState>("/hubomsg/HuboState") );
             RTT::types::Types()->addType( new types::PrimitiveSequenceTypeInfo<std::vector<hubomsg::HuboState> >("/hubomsg/HuboState[]") );
             RTT::types::Types()->addType( new types::CArrayTypeInfo<RTT::types::carray<hubomsg::HuboState> >("/hubomsg/cHuboState[]") );
        }

    
}

