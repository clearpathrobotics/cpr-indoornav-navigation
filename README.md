# cpr-indoornav-navigation
ROS action interface for programatically commanding an IndoorNav robot

This package contains a single ROS 1 node that is intended to wrap the Indoor Nav ROS 2 API's
`send_move_goal` example into a ROS 1 `actionlib` interface to make it easier to use IndoorNav
with ROS 1 Noetic.

## Services

The `indoornav_move_goal_node` provides 3 services for accessing waypoint & mission data defined
in the IndoorNav GUI:
- `get_markers`: get the list of all `Marker` objects defined in the map.
  `Marker` objects can be sent to the `move_to_marker` action, defined below
- `get_places`: an intermediate service used internally. `Place` objects contain a `primary_marker`
  (`Marker` type) object, and are used to store the list of `Task` objects executed by a `Mission`.
- `get_missions`: get the list of defined `Mission` objects ("Workflows") created in the GUI.
  `Mission` objects can be executed using the `execute_mission` action, defined below.

## Actions

The `indoornav_move_goal_node` provides 3 actions for driving autonomously:
- `move_to_location` accepts an arbitrary `(x, y, yaw`) position in meters (`x`, `y`) and radians (`yaw`)
  relative to the `map` frame. This action is a loose wrapper for the ROS 2 API's `send_move_goal`
  example, and uses the ROS 2 implementation internally.
- `move_to_marker` accepts a `Marker` object, and will drive the robot to the specified `Marker` on
  the map. Use this action if you want to manually step through a `Mission` to perform custom actions
  at a given location (e.g. drive to the `Pickup` marker and then execute a manipulation action using
  an arm and `MoveIt`)
- `execute_mission` takes a `Mission` object and executes it, exactly as if it were selected from the
  "Workflow" menu in the GUI. The robot will drive to each `Marker` defined in the `Mission`, one
  after the other.

## Installation & Launching

The `cpr_indoornav_navigation` and `cpr_indoornav_navigation_msgs` packages must be installed on
your Clearpath robot's primary computer.  (Jackal, Dingo-D, Dingo-O, Ridgeback, and Husky are all
supported).

To launch the node manually, run
```bash
roslaunch cpr_indoornav_navigation move_goal_node.launch
```

You may, if necessary, specify the hostname & TCP port of the IndoorNav computer if your configuration
requires this, e.g.:
```bash
roslaunch cpr_indoornav_navigation move_goal_node.launch hostname:=10.252.252.1 port:=5000
```

To start the node as part of IndoorNav, run
```bash
rosrun robot_upstart install --job cpr-indoornav --augment cpr_indoornav_navigation/launch/move_goal_node.launch
sudo systemctl daemon-reload
sudo systemctl restart cpr-indoornav
```

To access the actions and services from a remote workstation, install the `cpr_indoornav_navigation_msgs`
package. Make sure to set the `ROS_MASTER_URI` environment variable to the Clearpath robot's ROS master
as described in your robot's manual.