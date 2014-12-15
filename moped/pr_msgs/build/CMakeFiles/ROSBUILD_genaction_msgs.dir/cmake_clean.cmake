FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/pr_msgs/msg"
  "../src/pr_msgs/srv"
  "CMakeFiles/ROSBUILD_genaction_msgs"
  "../msg/SignalAction.msg"
  "../msg/SignalGoal.msg"
  "../msg/SignalActionGoal.msg"
  "../msg/SignalResult.msg"
  "../msg/SignalActionResult.msg"
  "../msg/SignalFeedback.msg"
  "../msg/SignalActionFeedback.msg"
  "../msg/herbAction.msg"
  "../msg/herbGoal.msg"
  "../msg/herbActionGoal.msg"
  "../msg/herbResult.msg"
  "../msg/herbActionResult.msg"
  "../msg/herbFeedback.msg"
  "../msg/herbActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
