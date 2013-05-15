/* Auto-generated by genmsg_cpp for file /opt/ros/fuerte/stacks/maestro/hubomsg/msg/HuboJointState.msg */
#ifndef HUBOMSG_BOOST_SERIALIZATION_HUBOJOINTSTATE_H
#define HUBOMSG_BOOST_SERIALIZATION_HUBOJOINTSTATE_H

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/nvp.hpp>
#include <hubomsg/HuboJointState.h>

namespace boost
{
namespace serialization
{

template<class Archive, class ContainerAllocator>
void serialize(Archive& a,  ::hubomsg::HuboJointState_<ContainerAllocator>  & m, unsigned int)
{
    a & make_nvp("name",m.name);
    a & make_nvp("commanded",m.commanded);
    a & make_nvp("position",m.position);
    a & make_nvp("velocity",m.velocity);
    a & make_nvp("current",m.current);
    a & make_nvp("temperature",m.temperature);
    a & make_nvp("active",m.active);
    a & make_nvp("zeroed",m.zeroed);
    a & make_nvp("homed",m.homed);
    a & make_nvp("status",m.status);
}

} // namespace serialization
} // namespace boost

#endif // HUBOMSG_BOOST_SERIALIZATION_HUBOJOINTSTATE_H

