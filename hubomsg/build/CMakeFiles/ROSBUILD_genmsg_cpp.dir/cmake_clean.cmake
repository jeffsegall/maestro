FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/hubomsg/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/hubomsg/HuboFT.h"
  "../msg_gen/cpp/include/hubomsg/HuboState.h"
  "../msg_gen/cpp/include/hubomsg/HuboHand.h"
  "../msg_gen/cpp/include/hubomsg/HuboIMU.h"
  "../msg_gen/cpp/include/hubomsg/AchCommand.h"
  "../msg_gen/cpp/include/hubomsg/HuboJointCommand.h"
  "../msg_gen/cpp/include/hubomsg/HuboHandCommand.h"
  "../msg_gen/cpp/include/hubomsg/CanMessage.h"
  "../msg_gen/cpp/include/hubomsg/HuboJointState.h"
  "../msg_gen/cpp/include/hubomsg/HuboCmd.h"
  "../msg_gen/cpp/include/hubomsg/HuboCommand.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
