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

| Name                             | Type                           | Description                                     |
| -------------------------------- | ------------------------------ | ----------------------------------------------- |
| planner/current_plan             | planner_msgs/PlanWithVel       | Current robot navigation plan with vels         |
| planner/current_plan_viz         | nav_msgs/Path                  | Current robot navigation plan for RVIZ display  |
| planner/goal_pose                | geometry_msgs/PoseStamped      | Goal position: plan to here                     |
| controller/wheel_estimate        | controller/WheelEst            | Encoder Information                             |
| controller/Imu                   | sensor_msgs/Imu                | Most recent IMU message                         |
| localizer/markers                | visualization_msgs/MarkerArray | Detected aruco markers                          | 
| state_estimator/joint_states     | sensor_msgs/JointStates        | Wheel angles and velocities (radians)           |

##### TF Tree

world -> camera_link
camera_link -> body_link
body_link -> wheelL;
body_link -> wheelR;
body_link -> imu_link;
world -> aruco_marker
