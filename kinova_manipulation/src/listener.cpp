#include "kinova_arm_control.hpp"
#include "std_msgs/Int32.h"
#include <stdlib.h>

class pub_sub{
	private:
		ros::Subscriber sub_arm;
	public:
		pub_sub(ros::NodeHandle& n){
			sub_arm = n.subscribe<std_msgs::Int32>("arm_start", 10, &pub_sub::callback, this); ROS_INFO("here");
		}

		void callback(const std_msgs::Int32::ConstPtr& begin){
			system("rosrun kinova_manipulation kinova_arm_planning");			
		}




};





int main(int argc, char** argv){
	ros::init(argc, argv, "listener");
	ros::NodeHandle node_handle;  
	pub_sub trial1(node_handle);
	//ros::AsyncSpinner spinner(0);
	//spinner.start();
	
	//KArmControl demo1(node_handle);
	//demo1.set_pose_goal(0.54882, -0.30854,  0.658410, .68463, -0.22436, 0.68808, 0.086576);
	//demo1.plan_goal();
	//demo1.set_pose_goal(0.74882, -0.30854,  0.658410, .68463, -0.22436, 0.68808, 0.086576);
	//demo1.plan_goal();
	//demo1.set_joint_goal(0,0,0,0,0,-1.0);
	//demo1.plan_goal();
	//demo1.add_object();
	//demo1.pick_and_place();
	//demo1.grasp_control(6400);
	/*
	demo1.set_pose_goal(0,-0.5,0.1,1,0,0,1);
	demo1.plan_goal();
	demo1.set_pose_goal(0,-0.7,0.1,1,0,0,1);
	demo1.plan_goal();
	demo1.grasp_control(3300);	
	demo1.set_pose_goal(.08,-0.7,0.15,1,0,0,1);
	demo1.plan_goal();
	demo1.grasp_control(0);	
	demo1.set_joint_goal(4.712,2.845,0,0.750,4.625,4.486,4.887);
	demo1.plan_goal();
	*/
	//ros::waitForShutdown();
	ros::spin();
}
