# cpr-indoornav-navigation
ROS action interface for programatically commanding an IndoorNav robot

This package contains a single ROS 1 node that is intended to wrap the Indoor Nav ROS 2 API's
`send_move_goal` example into a ROS 1 `actionlib` interface.

To launch the node, run
```bash
roslaunch cpr_indoornav_navigation move_goal_node.launch
```

To send a goal to the action server, either publish to the goal topic directly:
```bash
rostopic pub /indoornav_move_goal_node/send_move_goal/goal cpr_indoornav_navigation_msgs/MoveGoalActionGoal goal:
  x: 0.0
  y: 0.0
  orientation: 0.0" -1
```
where `x`, `y`, and `orientation` are passed directly to the ROS 2 API's `send_move_goal` executable.

Alternatively, you may also implement a complete `actionlib` client and send the goal using the
`cpr_indoornav_navigation_msgs/action/MoveGoal.action` messages.