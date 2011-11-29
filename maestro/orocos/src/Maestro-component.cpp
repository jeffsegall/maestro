#include "Maestro-component.hpp"
#include <rtt/Component.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

using namespace RTT;

class Maestro : public RTT::TaskContext
{
private:
  InputPort<std_msgs::Float64> inport;
  OutputPort<std_msgs::Float64> outport;

  InputPort<std_msgs::String> sinport;
  OutputPort<std_msgs::String> soutport;

};

ORO_CREATE_COMPONENT(Maestro)
