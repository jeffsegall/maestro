FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/hubomsg/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/hubomsg/msg/__init__.py"
  "../src/hubomsg/msg/_HuboFT.py"
  "../src/hubomsg/msg/_HuboState.py"
  "../src/hubomsg/msg/_HuboHand.py"
  "../src/hubomsg/msg/_HuboIMU.py"
  "../src/hubomsg/msg/_AchCommand.py"
  "../src/hubomsg/msg/_HuboJointCommand.py"
  "../src/hubomsg/msg/_HuboHandCommand.py"
  "../src/hubomsg/msg/_CanMessage.py"
  "../src/hubomsg/msg/_HuboJointState.py"
  "../src/hubomsg/msg/_HuboCmd.py"
  "../src/hubomsg/msg/_HuboCommand.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
