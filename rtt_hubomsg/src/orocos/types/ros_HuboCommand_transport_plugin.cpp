#include <hubomsg/boost/HuboCommand.h>
#include "ros_msg_transporter.hpp"
#include "RosLib.hpp"
#include <rtt/types/TransportPlugin.hpp>
#include <rtt/types/TypekitPlugin.hpp>

namespace ros_integration {
  using namespace RTT;
    struct ROSHuboCommandPlugin
      : public types::TransportPlugin
    {
      bool registerTransport(std::string name, types::TypeInfo* ti)
      {
	if(name == "/hubomsg/HuboCommand")
	  return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<hubomsg::HuboCommand>());
	return false;
      }
      
      std::string getTransportName() const {
	return "ros";
      }
      
      std::string getTypekitName() const {
	return std::string("ros-")+"HuboCommand";
      }
      std::string getName() const {
	return std::string("rtt-ros-") + "HuboCommand" + "-transport";
      }

    };
}

ORO_TYPEKIT_PLUGIN( ros_integration::ROSHuboCommandPlugin )
