#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <arm_control_pkg/StageAction.h>
#include "kinova_manipulation/kinova_arm_control.hpp"

class ManipulateActionServer{
	protected:	
		//variables for nodehandle, action server, feedback and results
		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<arm_control_pkg::StageAction> as_;
		std::string action_name_;
		arm_control_pkg::StageFeedback feedback_;
	    arm_control_pkg::StageResult result_;

	public:
		//Starts action server with name given in main function and begins async spinner for moveit
		ManipulateActionServer(std::string name):
		as_(nh_, name, boost::bind(&ManipulateActionServer::executeCB, this, _1), false)
		, action_name_(name){
			as_.start();
			ros::AsyncSpinner spinner(1);
			spinner.start();	
		}

		void executeCB(const arm_control_pkg::StageGoalConstPtr &goal){
			ROS_INFO("MANIPULATE ACTION EXECUTING");
			
			//checks for preempt
		 	if (as_.isPreemptRequested() || !ros::ok()){
        		ROS_INFO("%s: Preempted", action_name_.c_str());
        		as_.setPreempted();
      		}

			//if stage start is true execute manipulateing of arm if it completes succesfully return true as result
			//Needs error checking
			if(goal->stage_start == true){
				//creates object that has movegroup functions comment out and replace with your own code 
				KArmControl placeholder(nh_);
				placeholder.set_pose_goal(.08,-0.78,.1,1,0,0,1);
			    placeholder.plan_goal(); 		
			    placeholder.set_pose_goal(.15,-0.78,.13,1,0,0,1);
			    placeholder.plan_goal(); 
			    placeholder.set_pose_goal(.22,-0.78,.06,1,0,0,1);
			    placeholder.plan_goal(); 
	
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
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "manipulate_as");

  ManipulateActionServer demo("manipulate_as");
  ros::spin();

  return 0;
}
