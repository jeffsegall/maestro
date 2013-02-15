#include <hubomsg/boost/HuboIMU.h>
#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/StructTypeInfo.hpp>
#include <rtt/types/PrimitiveSequenceTypeInfo.hpp>
#include <rtt/types/CArrayTypeInfo.hpp>
#include <vector>

// Note: we need to put these up-front or we get gcc compiler warnings:
// <<warning: type attributes ignored after type is already defined>>        
template class RTT_EXPORT RTT::internal::DataSourceTypeInfo< hubomsg::HuboIMU >;
template class RTT_EXPORT RTT::internal::DataSource< hubomsg::HuboIMU >;
template class RTT_EXPORT RTT::internal::AssignableDataSource< hubomsg::HuboIMU >;
template class RTT_EXPORT RTT::internal::AssignCommand< hubomsg::HuboIMU >;
template class RTT_EXPORT RTT::internal::ValueDataSource< hubomsg::HuboIMU >;
template class RTT_EXPORT RTT::internal::ConstantDataSource< hubomsg::HuboIMU >;
template class RTT_EXPORT RTT::internal::ReferenceDataSource< hubomsg::HuboIMU >;
template class RTT_EXPORT RTT::OutputPort< hubomsg::HuboIMU >;
template class RTT_EXPORT RTT::InputPort< hubomsg::HuboIMU >;
template class RTT_EXPORT RTT::Property< hubomsg::HuboIMU >;
template class RTT_EXPORT RTT::Attribute< hubomsg::HuboIMU >;
template class RTT_EXPORT RTT::Constant< hubomsg::HuboIMU >;

namespace ros_integration {
  using namespace RTT;
    // Factory function
    
        void rtt_ros_addType_hubomsg_HuboIMU() {
             // Only the .msg type is sent over ports. The msg[] (variable size) and  cmsg[] (fixed size) exist only as members of larger messages
             RTT::types::Types()->addType( new types::StructTypeInfo<hubomsg::HuboIMU>("/hubomsg/HuboIMU") );
             RTT::types::Types()->addType( new types::PrimitiveSequenceTypeInfo<std::vector<hubomsg::HuboIMU> >("/hubomsg/HuboIMU[]") );
             RTT::types::Types()->addType( new types::CArrayTypeInfo<RTT::types::carray<hubomsg::HuboIMU> >("/hubomsg/cHuboIMU[]") );
        }

    
}

