FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/can_msgs/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/can_msgs/msg/__init__.py"
  "../src/can_msgs/msg/_CANFrame.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
