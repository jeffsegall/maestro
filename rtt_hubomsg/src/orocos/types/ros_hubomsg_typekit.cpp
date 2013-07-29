#include <hubomsg/HuboHandCommand.h>
#include <hubomsg/HuboFT.h>
#include <hubomsg/HuboJointCommand.h>
#include <hubomsg/HuboCmd.h>
#include <hubomsg/HuboJointState.h>
#include <hubomsg/HuboHand.h>
#include <hubomsg/HuboState.h>
#include <hubomsg/CanMessage.h>
#include <hubomsg/HuboCommand.h>
#include <hubomsg/AchCommand.h>
#include <hubomsg/HuboIMU.h>

#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/StructTypeInfo.hpp>

namespace ros_integration {
  using namespace RTT;

    /** Declare all factory functions */
            void rtt_ros_addType_hubomsg_HuboHandCommand();
        void rtt_ros_addType_hubomsg_HuboFT();
        void rtt_ros_addType_hubomsg_HuboJointCommand();
        void rtt_ros_addType_hubomsg_HuboCmd();
        void rtt_ros_addType_hubomsg_HuboJointState();
        void rtt_ros_addType_hubomsg_HuboHand();
        void rtt_ros_addType_hubomsg_HuboState();
        void rtt_ros_addType_hubomsg_CanMessage();
        void rtt_ros_addType_hubomsg_HuboCommand();
        void rtt_ros_addType_hubomsg_AchCommand();
        void rtt_ros_addType_hubomsg_HuboIMU();

   
    /**
     * This interface defines the types of the realTime package.
     */
    class ROShubomsgTypekitPlugin
      : public types::TypekitPlugin
    {
    public:
      virtual std::string getName(){
          return std::string("ros-")+"hubomsg";
      }

      virtual bool loadTypes() {
          // call all factory functions
                  rtt_ros_addType_hubomsg_HuboHandCommand(); // factory function for adding TypeInfo.
        rtt_ros_addType_hubomsg_HuboFT(); // factory function for adding TypeInfo.
        rtt_ros_addType_hubomsg_HuboJointCommand(); // factory function for adding TypeInfo.
        rtt_ros_addType_hubomsg_HuboCmd(); // factory function for adding TypeInfo.
        rtt_ros_addType_hubomsg_HuboJointState(); // factory function for adding TypeInfo.
        rtt_ros_addType_hubomsg_HuboHand(); // factory function for adding TypeInfo.
        rtt_ros_addType_hubomsg_HuboState(); // factory function for adding TypeInfo.
        rtt_ros_addType_hubomsg_CanMessage(); // factory function for adding TypeInfo.
        rtt_ros_addType_hubomsg_HuboCommand(); // factory function for adding TypeInfo.
        rtt_ros_addType_hubomsg_AchCommand(); // factory function for adding TypeInfo.
        rtt_ros_addType_hubomsg_HuboIMU(); // factory function for adding TypeInfo.

          return true;
      }
      virtual bool loadOperators() { return true; }
      virtual bool loadConstructors() { return true; }
    };
}

ORO_TYPEKIT_PLUGIN( ros_integration::ROShubomsgTypekitPlugin )

