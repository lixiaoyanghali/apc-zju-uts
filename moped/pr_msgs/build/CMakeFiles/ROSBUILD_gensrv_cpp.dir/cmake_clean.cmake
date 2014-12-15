FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/pr_msgs/msg"
  "../src/pr_msgs/srv"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/pr_msgs/ResumeTrajectory.h"
  "../srv_gen/cpp/include/pr_msgs/Idle.h"
  "../srv_gen/cpp/include/pr_msgs/Enable.h"
  "../srv_gen/cpp/include/pr_msgs/AskUser.h"
  "../srv_gen/cpp/include/pr_msgs/ArmConfigCheck.h"
  "../srv_gen/cpp/include/pr_msgs/AppletCommand.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
