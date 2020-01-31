#include "planner/planner_node.h"


bool make_plan_msg(std::vector<double> px, std::vector<double> py, std::vector<double> theta,
                      std::vector<double> vx, std::vector<double> vy, std::vector<double> omega,
                      std::vector<double> ts,
                      PlanMsg &plan_msg, nav_msgs::Path &viz_msg)
{
  plan_msg.header.stamp = ros::Time::now();
  plan_msg.header.frame_id = "world";
  for (size_t i = 0; i < px.size(); ++i)
  {
    ///////////// Position ///////////////////

    // Translation part (leave z component 0, 2D system)
    Pose p;
    p.position.x = px.at(i);
    p.position.y = py.at(i);;
    p.position.z = 0;
    
    // Orientation part (should be yaw only)
    tf2::Quaternion quat_tf2;
    quat_tf2.setRPY( 0, 0, theta.at(i));
    tf2::convert(p.orientation , quat_tf2);
    plan_msg.poses.push_back(p);

    ///////////// Visualization Only! /////////
    PoseStamped ps;
    ps.pose = p;
    ps.header.stamp = ros::Time::now();
    ps.header.frame_id = "world";
    viz_msg.poses.push_back(ps);

    //////////// Velocity ////////////////
    Twist t;
    // Linear Velocity (leave z component 0, 2D system)
    t.linear.x = vx.at(i);
    t.linear.y = vy.at(i);
    t.linear.z = 0;

    // Angular Velocity (leave x,y components 0, 2D system)
    t.angular.x = 0;
    t.angular.y = 0;
    t.angular.z = omega.at(i);
    plan_msg.twists.push_back(t);
  }
  return true;
}