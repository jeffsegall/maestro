#include <hubomsg/boost/AchCommand.h>
#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/StructTypeInfo.hpp>
#include <rtt/types/PrimitiveSequenceTypeInfo.hpp>
#include <rtt/types/CArrayTypeInfo.hpp>
#include <vector>

// Note: we need to put these up-front or we get gcc compiler warnings:
// <<warning: type attributes ignored after type is already defined>>        
template class RTT_EXPORT RTT::internal::DataSourceTypeInfo< hubomsg::AchCommand >;
template class RTT_EXPORT RTT::internal::DataSource< hubomsg::AchCommand >;
template class RTT_EXPORT RTT::internal::AssignableDataSource< hubomsg::AchCommand >;
template class RTT_EXPORT RTT::internal::AssignCommand< hubomsg::AchCommand >;
template class RTT_EXPORT RTT::internal::ValueDataSource< hubomsg::AchCommand >;
template class RTT_EXPORT RTT::internal::ConstantDataSource< hubomsg::AchCommand >;
template class RTT_EXPORT RTT::internal::ReferenceDataSource< hubomsg::AchCommand >;
template class RTT_EXPORT RTT::OutputPort< hubomsg::AchCommand >;
template class RTT_EXPORT RTT::InputPort< hubomsg::AchCommand >;
template class RTT_EXPORT RTT::Property< hubomsg::AchCommand >;
template class RTT_EXPORT RTT::Attribute< hubomsg::AchCommand >;
template class RTT_EXPORT RTT::Constant< hubomsg::AchCommand >;

namespace ros_integration {
  using namespace RTT;
    // Factory function
    
        void rtt_ros_addType_hubomsg_AchCommand() {
             // Only the .msg type is sent over ports. The msg[] (variable size) and  cmsg[] (fixed size) exist only as members of larger messages
             RTT::types::Types()->addType( new types::StructTypeInfo<hubomsg::AchCommand>("/hubomsg/AchCommand") );
             RTT::types::Types()->addType( new types::PrimitiveSequenceTypeInfo<std::vector<hubomsg::AchCommand> >("/hubomsg/AchCommand[]") );
             RTT::types::Types()->addType( new types::CArrayTypeInfo<RTT::types::carray<hubomsg::AchCommand> >("/hubomsg/cAchCommand[]") );
        }

    
}

