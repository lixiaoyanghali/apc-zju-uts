FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/pr_msgs/msg"
  "../src/pr_msgs/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/pr_msgs/srv/__init__.py"
  "../src/pr_msgs/srv/_ResumeTrajectory.py"
  "../src/pr_msgs/srv/_Idle.py"
  "../src/pr_msgs/srv/_Enable.py"
  "../src/pr_msgs/srv/_AskUser.py"
  "../src/pr_msgs/srv/_ArmConfigCheck.py"
  "../src/pr_msgs/srv/_AppletCommand.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
