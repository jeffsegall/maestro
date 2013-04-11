#include <hubomsg/boost/HuboCommand.h>
#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/StructTypeInfo.hpp>
#include <rtt/types/PrimitiveSequenceTypeInfo.hpp>
#include <rtt/types/CArrayTypeInfo.hpp>
#include <vector>

// Note: we need to put these up-front or we get gcc compiler warnings:
// <<warning: type attributes ignored after type is already defined>>        
template class RTT_EXPORT RTT::internal::DataSourceTypeInfo< hubomsg::HuboCommand >;
template class RTT_EXPORT RTT::internal::DataSource< hubomsg::HuboCommand >;
template class RTT_EXPORT RTT::internal::AssignableDataSource< hubomsg::HuboCommand >;
template class RTT_EXPORT RTT::internal::AssignCommand< hubomsg::HuboCommand >;
template class RTT_EXPORT RTT::internal::ValueDataSource< hubomsg::HuboCommand >;
template class RTT_EXPORT RTT::internal::ConstantDataSource< hubomsg::HuboCommand >;
template class RTT_EXPORT RTT::internal::ReferenceDataSource< hubomsg::HuboCommand >;
template class RTT_EXPORT RTT::OutputPort< hubomsg::HuboCommand >;
template class RTT_EXPORT RTT::InputPort< hubomsg::HuboCommand >;
template class RTT_EXPORT RTT::Property< hubomsg::HuboCommand >;
template class RTT_EXPORT RTT::Attribute< hubomsg::HuboCommand >;
template class RTT_EXPORT RTT::Constant< hubomsg::HuboCommand >;

namespace ros_integration {
  using namespace RTT;
    // Factory function
    
        void rtt_ros_addType_hubomsg_HuboCommand() {
             // Only the .msg type is sent over ports. The msg[] (variable size) and  cmsg[] (fixed size) exist only as members of larger messages
             RTT::types::Types()->addType( new types::StructTypeInfo<hubomsg::HuboCommand>("/hubomsg/HuboCommand") );
             RTT::types::Types()->addType( new types::PrimitiveSequenceTypeInfo<std::vector<hubomsg::HuboCommand> >("/hubomsg/HuboCommand[]") );
             RTT::types::Types()->addType( new types::CArrayTypeInfo<RTT::types::carray<hubomsg::HuboCommand> >("/hubomsg/cHuboCommand[]") );
        }

    
}

