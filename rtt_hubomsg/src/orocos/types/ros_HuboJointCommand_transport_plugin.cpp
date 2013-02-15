#include <hubomsg/boost/HuboJointCommand.h>
#include "ros_msg_transporter.hpp"
#include "RosLib.hpp"
#include <rtt/types/TransportPlugin.hpp>
#include <rtt/types/TypekitPlugin.hpp>

namespace ros_integration {
  using namespace RTT;
    struct ROSHuboJointCommandPlugin
      : public types::TransportPlugin
    {
      bool registerTransport(std::string name, types::TypeInfo* ti)
      {
	if(name == "/hubomsg/HuboJointCommand")
	  return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<hubomsg::HuboJointCommand>());
	return false;
      }
      
      std::string getTransportName() const {
	return "ros";
      }
      
      std::string getTypekitName() const {
	return std::string("ros-")+"HuboJointCommand";
      }
      std::string getName() const {
	return std::string("rtt-ros-") + "HuboJointCommand" + "-transport";
      }

    };
}

ORO_TYPEKIT_PLUGIN( ros_integration::ROSHuboJointCommandPlugin )
