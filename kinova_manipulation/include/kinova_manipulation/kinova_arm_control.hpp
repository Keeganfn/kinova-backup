#ifndef KINOVA_ARM_CONTROL_HPP
#define KINOVA_ARM_CONTROL_HPP


#include <boost/filesystem.hpp>
#include "ros/ros.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h> 
#include <kinova_driver/kinova_ros_types.h>
#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/SetFingersPositionAction.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/collision_detection/collision_tools.h>
//#include <moveit/collision_detection_fcl/collision_env_fcl.h>
typedef boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MGP;
typedef boost::shared_ptr<moveit::planning_interface::PlanningSceneInterface> PSP;
typedef boost::shared_ptr<actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>> FPA;
class KArmControl{
	
	private:
	ros::NodeHandle nh;
	ros::Publisher display_publisher;
	moveit_msgs::DisplayTrajectory display_trajectory;
	moveit_msgs::CollisionObject collision_object;


	public:
	
	MGP move_group_ptr;
	MGP finger_group_ptr;
	PSP planning_scene_ptr;
	FPA finger_action_client_ptr;
	bool robot_connected_;
  robot_model_loader::RobotModelLoader rm_loader_;
  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;

	KArmControl(ros::NodeHandle& node_handle);
	void set_pose_goal(float vx, float vy, float vz, float w_val, float x_val, float y_val, float z_val);
	void set_joint_goal(double j1, double j2, double j3, double j4, double j5, double j6, double j7);
	void add_object();
	bool grasp_control(float finger_position);
	void plan_goal();
	void pick_and_place();
};


#endif
