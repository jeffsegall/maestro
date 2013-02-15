
#include <hubomsg/HuboHand.h>
#include <hubomsg/HuboJointState.h>
#include <hubomsg/HuboHandCommand.h>
#include <hubomsg/HuboCmd.h>
#include <hubomsg/HuboJointCommand.h>
#include <hubomsg/HuboCommand.h>
#include <hubomsg/HuboState.h>
#include <hubomsg/CanMessage.h>
#include <hubomsg/HuboFT.h>
#include <hubomsg/HuboIMU.h>

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
                   if(name == "/hubomsg/HuboHand")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<hubomsg::HuboHand>());
         if(name == "/hubomsg/HuboJointState")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<hubomsg::HuboJointState>());
         if(name == "/hubomsg/HuboHandCommand")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<hubomsg::HuboHandCommand>());
         if(name == "/hubomsg/HuboCmd")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<hubomsg::HuboCmd>());
         if(name == "/hubomsg/HuboJointCommand")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<hubomsg::HuboJointCommand>());
         if(name == "/hubomsg/HuboCommand")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<hubomsg::HuboCommand>());
         if(name == "/hubomsg/HuboState")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<hubomsg::HuboState>());
         if(name == "/hubomsg/CanMessage")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<hubomsg::CanMessage>());
         if(name == "/hubomsg/HuboFT")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<hubomsg::HuboFT>());
         if(name == "/hubomsg/HuboIMU")
              return ti->addProtocol(ORO_ROS_PROTOCOL_ID,new RosMsgTransporter<hubomsg::HuboIMU>());

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
