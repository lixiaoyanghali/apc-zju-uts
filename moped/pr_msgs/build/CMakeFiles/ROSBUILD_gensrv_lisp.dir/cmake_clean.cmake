FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/pr_msgs/msg"
  "../src/pr_msgs/srv"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/ResumeTrajectory.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_ResumeTrajectory.lisp"
  "../srv_gen/lisp/Idle.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_Idle.lisp"
  "../srv_gen/lisp/Enable.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_Enable.lisp"
  "../srv_gen/lisp/AskUser.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_AskUser.lisp"
  "../srv_gen/lisp/ArmConfigCheck.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_ArmConfigCheck.lisp"
  "../srv_gen/lisp/AppletCommand.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_AppletCommand.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
