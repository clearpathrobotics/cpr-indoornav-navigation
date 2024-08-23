# cpr-indoornav-navigation
ROS action interface for programatically commanding an IndoorNav robot

This package contains a single ROS 1 node that is intended to wrap the Indoor Nav ROS 2 API's
`send_move_goal` example into a ROS 1 `actionlib` interface.

To launch the node, run
```bash
roslaunch cpr_indoornav_navigation move_goal_node.launch
```

To get the list of Waypoint markers on the map, use the `get_markers` service:
```bash
$ rosservice call /indoornav_move_goal_node/get_markers
markers: 
  - 
    id: "92bc7b80-5807-4e93-95ea-3b9a9d23525d"
    name: "Laura's desk"
    type: "Feature"
    item_type: "WAYPOINT"
    marker_intent: "WAYPOINT"
    notes: ''
    yaw: 0.15
    x: 2.3113582259634384
    y: -3.0388987205631146
  - 
    id: "7a6f0246-0cc7-4a11-92f0-bd7f0b140ad2"
    name: "Lobby"
    type: "Feature"
    item_type: "WAYPOINT"
    marker_intent: "WAYPOINT"
    notes: ''
    yaw: 1.7031853071795864
    x: 11.684468932327853
    y: 1.299512520668415
  - 
    id: "b17d2513-1dd1-417e-b841-48600fb688ea"
    name: "Jason's desk"
    type: "Feature"
    item_type: "WAYPOINT"
    marker_intent: "WAYPOINT"
    notes: ''
    yaw: -1.4452072139277519
    x: 0.7808958607071048
    y: 8.084568910849924
  - 
    id: "9a5ce76e-1284-4e47-b912-772a76835ddd"
    name: "Water cooler"
    type: "Feature"
    item_type: "WAYPOINT"
    marker_intent: "WAYPOINT"
    notes: ''
    yaw: -3.0
    x: 8.023395130041493
    y: 9.334741471330926
  - 
    id: "dfe23aed-96a3-417e-8c83-cd0128d8d477"
    name: "Caffeteria"
    type: "Feature"
    item_type: "WAYPOINT"
    marker_intent: "WAYPOINT"
    notes: ''
    yaw: -1.06
    x: 3.132591873891343
    y: 21.010673188478115
```

These markers can be collected into missions/workflows. These can be queried with the `get_missions`
service:
```bash

```

To send a goal to the action server, either publish to the goal topic directly, or implement an
`actionlib` client that uses the `MoveGoal` action:
```bash
rostopic pub /indoornav_move_goal_node/send_move_goal/goal cpr_indoornav_navigation_msgs/MoveGoalActionGoal goal:
  destination:
    id: "dfe23aed-96a3-417e-8c83-cd0128d8d477"
    name: "Caffeteria"
    type: "Feature"
    item_type: "WAYPOINT"
    marker_intent: "WAYPOINT"
    notes: ''
    yaw: -1.06
    x: 3.132591873891343
    y: 21.010673188478115" -1
```
where `destination.x`, `destination.y`, and `destination.yaw` are passed directly to the ROS 2 API's
`send_move_goal` executable.