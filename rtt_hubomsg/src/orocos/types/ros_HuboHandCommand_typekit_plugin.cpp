#include <hubomsg/boost/HuboHandCommand.h>
#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/StructTypeInfo.hpp>
#include <rtt/types/PrimitiveSequenceTypeInfo.hpp>
#include <rtt/types/CArrayTypeInfo.hpp>
#include <vector>

// Note: we need to put these up-front or we get gcc compiler warnings:
// <<warning: type attributes ignored after type is already defined>>        
template class RTT_EXPORT RTT::internal::DataSourceTypeInfo< hubomsg::HuboHandCommand >;
template class RTT_EXPORT RTT::internal::DataSource< hubomsg::HuboHandCommand >;
template class RTT_EXPORT RTT::internal::AssignableDataSource< hubomsg::HuboHandCommand >;
template class RTT_EXPORT RTT::internal::AssignCommand< hubomsg::HuboHandCommand >;
template class RTT_EXPORT RTT::internal::ValueDataSource< hubomsg::HuboHandCommand >;
template class RTT_EXPORT RTT::internal::ConstantDataSource< hubomsg::HuboHandCommand >;
template class RTT_EXPORT RTT::internal::ReferenceDataSource< hubomsg::HuboHandCommand >;
template class RTT_EXPORT RTT::OutputPort< hubomsg::HuboHandCommand >;
template class RTT_EXPORT RTT::InputPort< hubomsg::HuboHandCommand >;
template class RTT_EXPORT RTT::Property< hubomsg::HuboHandCommand >;
template class RTT_EXPORT RTT::Attribute< hubomsg::HuboHandCommand >;
template class RTT_EXPORT RTT::Constant< hubomsg::HuboHandCommand >;

namespace ros_integration {
  using namespace RTT;
    // Factory function
    
        void rtt_ros_addType_hubomsg_HuboHandCommand() {
             // Only the .msg type is sent over ports. The msg[] (variable size) and  cmsg[] (fixed size) exist only as members of larger messages
             RTT::types::Types()->addType( new types::StructTypeInfo<hubomsg::HuboHandCommand>("/hubomsg/HuboHandCommand") );
             RTT::types::Types()->addType( new types::PrimitiveSequenceTypeInfo<std::vector<hubomsg::HuboHandCommand> >("/hubomsg/HuboHandCommand[]") );
             RTT::types::Types()->addType( new types::CArrayTypeInfo<RTT::types::carray<hubomsg::HuboHandCommand> >("/hubomsg/cHuboHandCommand[]") );
        }

    
}

