#include <hubomsg/boost/HuboCmd.h>
#include "ros_msg_transporter.hpp"
#include "RosLib.hpp"
#include <rtt/types/TransportPlugin.hpp>
#include <rtt/types/TypekitPlugin.hpp>

namespace ros_integration {
  using namespace RTT;
    struct ROSHuboCmdPlugin
      : public types::TransportPlugin
    {
      bool registerTransport(std::string name, types::TypeInfo* ti)
      {
	if(name == "/hubomsg/HuboCmd")
	  return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<hubomsg::HuboCmd>());
	return false;
      }
      
      std::string getTransportName() const {
	return "ros";
      }
      
      std::string getTypekitName() const {
	return std::string("ros-")+"HuboCmd";
      }
      std::string getName() const {
	return std::string("rtt-ros-") + "HuboCmd" + "-transport";
      }

    };
}

ORO_TYPEKIT_PLUGIN( ros_integration::ROSHuboCmdPlugin )
