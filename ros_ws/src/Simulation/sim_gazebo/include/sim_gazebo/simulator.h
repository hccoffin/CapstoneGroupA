#ifndef ROBOASSISTANT_SIMULATOR_H
#define ROBOASSISTANT_SIMULATOR_H


#include <ros/ros.h>
#include <string>
#include <vector>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

namespace gazebo
{
class joint;
class entity;

class RoboAssistant
{
public:
	RoboAssistant();
	~RoboAssistant();
	void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
	void Reset();

private:
	GazeboRosPtr gazebo_ros_;
	physics::ModelPtr parent_;
	sdf::ElementPtr sdf_;
	event::ConnectionPtr update_connection_;

};

}

#endif
