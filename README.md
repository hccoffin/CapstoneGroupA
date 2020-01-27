# CapstoneGroupA


### Setup Instructions

```bash
cd ~
git clone --recursive https://github.com/hccoffin/CapstoneGroupA.git
cd CapstoneGroupA
chmod +x setup.sh && ./setup.sh
```

### ROS Implementation Specifics (tentative)

##### Topics
All are pre-pended with the name of their parent package

| Name                             | Type                           | Description                           |
| -------------------------------- | ------------------------------ | ------------------------------------- |
| planner/current_plan             | nav_msgs/Path                  | Current robot navigation plan         |
| planner/goal_pose                | geometry_msgs/PoseStamped      | Goal position: plan to here           |
| localizer/markers                | visualization_msgs/MarkerArray | Detected aruco markers                |
| state_estimator/joint_states     | sensor_msgs/JointStates        | Wheel angles and velocities (radians) |

##### TF Tree
world->base_link

world->aruco_marker
