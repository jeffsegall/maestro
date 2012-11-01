
#include <hubomsg/HuboCmd.h>
#include <hubomsg/CanMessage.h>

#include "ros_msg_transporter.hpp"
#include "RosLib.hpp"
#include <rtt/types/TransportPlugin.hpp>
#include <rtt/types/TypekitPlugin.hpp>

namespace ros_integration {
  using namespace RTT;
    struct ROShubomsgPlugin
      : public types::TransportPlugin
    {
      bool registerTransport(std::string name, types::TypeInfo* ti)
      {
                   if(name == "/hubomsg/HuboCmd")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<hubomsg::HuboCmd>());
         if(name == "/hubomsg/CanMessage")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<hubomsg::CanMessage>());

          return false;
      }
      
      std::string getTransportName() const {
          return "ros";
      }
      
      std::string getTypekitName() const {
          return std::string("ros-")+"hubomsg";
      }
      std::string getName() const {
          return std::string("rtt-ros-") + "hubomsg" + "-transport";
      }

    };
}

ORO_TYPEKIT_PLUGIN( ros_integration::ROShubomsgPlugin )
