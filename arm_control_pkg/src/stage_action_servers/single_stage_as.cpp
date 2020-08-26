#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <arm_control_pkg/StageAction.h>
#include "kinova_manipulation/kinova_arm_control.hpp"
class SingleStageActionServer{
	protected:	
		//variables for nodehandle, action server, feedback and results
		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<arm_control_pkg::StageAction> as_;
		std::string action_name_;
		arm_control_pkg::StageFeedback feedback_;
	    arm_control_pkg::StageResult result_;

	public:
		//Starts action server with name given in main function and begins async spinner for moveit
		SingleStageActionServer(std::string name):
		as_(nh_, name, boost::bind(&SingleStageActionServer::executeCB, this, _1), false)
		, action_name_(name){
			as_.start();
			ros::AsyncSpinner spinner(1);
			spinner.start();	
		}

		void executeCB(const arm_control_pkg::StageGoalConstPtr &goal){
			ROS_INFO("POSITION ACTION EXECUTING");
			
			//checks for preempt
		 	if (as_.isPreemptRequested() || !ros::ok()){
        		ROS_INFO("%s: Preempted", action_name_.c_str());
        		as_.setPreempted();
      		}

			//if stage start is true execute singlestageing of arm if it completes succesfully return true as result
			//Needs error checking
			if(goal->stage_start == true){
				//creates object that has movegroup functions comment out and replace with your own code 
				KArmControl placeholder(nh_);
				placeholder.add_object();
				placeholder.grasp_control(1); 
				placeholder.set_pose_goal(0,-0.5,.5,1,0,0,1);
				placeholder.plan_goal();
				
				while(!computeCollisionContactPoints(placeholder)){
					ROS_INFO("CHECKING");
				}

				ROS_INFO("SUCCESS");
				result_.stage_result = true;
				as_.setSucceeded(result_);
			}
			else{
				ROS_INFO("FAILED");
				result_.stage_result = false;
				as_.setSucceeded(result_);
			}
		}


	int computeCollisionContactPoints(KArmControl& robot){
		  collision_detection::CollisionRequest c_req;
		  collision_detection::CollisionResult c_res;
		  c_req.group_name = robot.move_group_ptr->getName();
		  c_req.contacts = true;
		  c_req.max_contacts = 100;
		  c_req.max_contacts_per_pair = 5;
		  c_req.verbose = false;

		  // Checking for Collisions
		  // ^^^^^^^^^^^^^^^^^^^^^^^
		  // We check for collisions between robot and itself or the world.
//		  robot.planning_scene_ptr->checkCollision(c_req, c_res, *robot.robot_model_);

		  // Displaying Collision Contact Points
		  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
		  // If there are collisions, we get the contact points and display them as markers.
		  // **getCollisionMarkersFromContacts()** is a helper function that adds the
		  // collision contact points into a MarkerArray message. If you want to use
		  // the contact points for something other than displaying them you can
		  // iterate through **c_res.contacts** which is a std::map of contact points.
		  // Look at the implementation of getCollisionMarkersFromContacts() in
		  // `collision_tools.cpp
		  // <https://github.com/ros-planning/moveit/blob/melodic-devel/moveit_core/collision_detection/src/collision_tools.cpp>`_
		  // for how.
		  if (c_res.collision)
		  {
			ROS_INFO("COLLIDING contact_point_count=%d", (int)c_res.contact_count);
			return 1;
		  }
		  // END_SUB_TUTORIAL
		  else
		  {
			ROS_INFO("Not colliding");
			return 0;
		  }
		}

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "singlestage_as");

  SingleStageActionServer demo("singlestage_as");
  ros::spin();

  return 0;
}
