#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <arm_control_pkg/StageAction.h>
#include "kinova_manipulation/kinova_arm_control.hpp"

class GraspActionServer{
	protected:	
		//variables for nodehandle, action server, feedback and results
		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<arm_control_pkg::StageAction> as_;
		std::string action_name_;
		arm_control_pkg::StageFeedback feedback_;
	    arm_control_pkg::StageResult result_;

	public:
		//Starts action server with name given in main function and begins async spinner for moveit
		GraspActionServer(std::string name):
		as_(nh_, name, boost::bind(&GraspActionServer::executeCB, this, _1), false)
		, action_name_(name){
			as_.start();
			ros::AsyncSpinner spinner(1);
			spinner.start();	
		}

		void executeCB(const arm_control_pkg::StageGoalConstPtr &goal){
			ROS_INFO("GRASP ACTION EXECUTING");
			
			//checks for preempt
		 	if (as_.isPreemptRequested() || !ros::ok()){
        		ROS_INFO("%s: Preempted", action_name_.c_str());
        		as_.setPreempted();
      		}

			//if stage start is true execute positioning of arm if it completes succesfully return true as result
			//Needs error checking
			if(goal->stage_start == true){
				//creates object that has movegroup functions comment out and replace with your own code 
				KArmControl placeholder(nh_);
   				placeholder.grasp_control(5900);  

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
  ros::init(argc, argv, "grasp_as");

  GraspActionServer demo("grasp_as");
  ros::spin();

  return 0;
}
