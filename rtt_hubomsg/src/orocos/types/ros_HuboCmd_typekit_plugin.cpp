#include <hubomsg/boost/HuboCmd.h>
#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/StructTypeInfo.hpp>
#include <rtt/types/PrimitiveSequenceTypeInfo.hpp>
#include <rtt/types/CArrayTypeInfo.hpp>
#include <vector>

// Note: we need to put these up-front or we get gcc compiler warnings:
// <<warning: type attributes ignored after type is already defined>>        
template class RTT_EXPORT RTT::internal::DataSourceTypeInfo< hubomsg::HuboCmd >;
template class RTT_EXPORT RTT::internal::DataSource< hubomsg::HuboCmd >;
template class RTT_EXPORT RTT::internal::AssignableDataSource< hubomsg::HuboCmd >;
template class RTT_EXPORT RTT::internal::AssignCommand< hubomsg::HuboCmd >;
template class RTT_EXPORT RTT::internal::ValueDataSource< hubomsg::HuboCmd >;
template class RTT_EXPORT RTT::internal::ConstantDataSource< hubomsg::HuboCmd >;
template class RTT_EXPORT RTT::internal::ReferenceDataSource< hubomsg::HuboCmd >;
template class RTT_EXPORT RTT::OutputPort< hubomsg::HuboCmd >;
template class RTT_EXPORT RTT::InputPort< hubomsg::HuboCmd >;
template class RTT_EXPORT RTT::Property< hubomsg::HuboCmd >;
template class RTT_EXPORT RTT::Attribute< hubomsg::HuboCmd >;
template class RTT_EXPORT RTT::Constant< hubomsg::HuboCmd >;

namespace ros_integration {
  using namespace RTT;
    // Factory function
    
        void rtt_ros_addType_hubomsg_HuboCmd() {
             // Only the .msg type is sent over ports. The msg[] (variable size) and  cmsg[] (fixed size) exist only as members of larger messages
             RTT::types::Types()->addType( new types::StructTypeInfo<hubomsg::HuboCmd>("/hubomsg/HuboCmd") );
             RTT::types::Types()->addType( new types::PrimitiveSequenceTypeInfo<std::vector<hubomsg::HuboCmd> >("/hubomsg/HuboCmd[]") );
             RTT::types::Types()->addType( new types::CArrayTypeInfo<RTT::types::carray<hubomsg::HuboCmd> >("/hubomsg/cHuboCmd[]") );
        }

    
}

