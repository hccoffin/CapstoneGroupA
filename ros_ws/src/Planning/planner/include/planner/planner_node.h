#ifndef ROBOASSISTANT_PLANNER_H
#define ROBOASSISTANT_PLANNER_H

#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <planner_msgs/PlanWithVel.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

typedef nav_msgs::Path VizMsg;
typedef geometry_msgs::PoseStamped PoseStamped;

typedef planner_msgs::PlanWithVel PlanMsg;
typedef geometry_msgs::Pose Pose;
typedef geometry_msgs::Twist Twist;


bool make_plan_msg(std::vector<double> px, std::vector<double> py, std::vector<double> theta,
                      std::vector<double> vx, std::vector<double> vy, std::vector<double> omega,
                      PlanMsg &plan_msg, VizMsg &viz_msg);

#endif