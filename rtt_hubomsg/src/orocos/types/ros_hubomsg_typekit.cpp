#include <hubomsg/HuboCmd.h>
#include <hubomsg/CanMessage.h>

#include <rtt/types/TypekitPlugin.hpp>
#include <rtt/types/StructTypeInfo.hpp>

namespace ros_integration {
  using namespace RTT;

    /** Declare all factory functions */
            void rtt_ros_addType_HuboCmd();
        void rtt_ros_addType_CanMessage();

   
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
                  rtt_ros_addType_HuboCmd(); // factory function for adding TypeInfo.
        rtt_ros_addType_CanMessage(); // factory function for adding TypeInfo.

          return true;
      }
      virtual bool loadOperators() { return true; }
      virtual bool loadConstructors() { return true; }
    };
}

ORO_TYPEKIT_PLUGIN( ros_integration::ROShubomsgTypekitPlugin )

