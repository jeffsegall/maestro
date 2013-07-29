FILE(REMOVE_RECURSE
  "../src/orocos/types/ros_HuboFT_typekit_plugin.cpp"
  "../src/orocos/types/ros_HuboState_typekit_plugin.cpp"
  "../src/orocos/types/ros_HuboHand_typekit_plugin.cpp"
  "../src/orocos/types/ros_HuboIMU_typekit_plugin.cpp"
  "../src/orocos/types/ros_AchCommand_typekit_plugin.cpp"
  "../src/orocos/types/ros_HuboJointCommand_typekit_plugin.cpp"
  "../src/orocos/types/ros_HuboHandCommand_typekit_plugin.cpp"
  "../src/orocos/types/ros_CanMessage_typekit_plugin.cpp"
  "../src/orocos/types/ros_HuboJointState_typekit_plugin.cpp"
  "../src/orocos/types/ros_HuboCmd_typekit_plugin.cpp"
  "../src/orocos/types/ros_HuboCommand_typekit_plugin.cpp"
  "../src/orocos/types/ros_HuboFT_transport_plugin.cpp"
  "../src/orocos/types/ros_HuboState_transport_plugin.cpp"
  "../src/orocos/types/ros_HuboHand_transport_plugin.cpp"
  "../src/orocos/types/ros_HuboIMU_transport_plugin.cpp"
  "../src/orocos/types/ros_AchCommand_transport_plugin.cpp"
  "../src/orocos/types/ros_HuboJointCommand_transport_plugin.cpp"
  "../src/orocos/types/ros_HuboHandCommand_transport_plugin.cpp"
  "../src/orocos/types/ros_CanMessage_transport_plugin.cpp"
  "../src/orocos/types/ros_HuboJointState_transport_plugin.cpp"
  "../src/orocos/types/ros_HuboCmd_transport_plugin.cpp"
  "../src/orocos/types/ros_HuboCommand_transport_plugin.cpp"
  "../src/orocos/types/ros_hubomsg_typekit.cpp"
  "../src/orocos/types/ros_hubomsg_transport.cpp"
  "../include/hubomsg/boost"
  "CMakeFiles/rospack_genmsg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/rospack_genmsg.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
