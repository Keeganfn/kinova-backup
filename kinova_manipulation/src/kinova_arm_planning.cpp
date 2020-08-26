#include "kinova_arm_control.hpp"
#include "std_msgs/Int32.h"
#include <stdlib.h>
/*
class pub_sub{
	private:
		ros::Publisher pub_;
		ros::Subscriber sub_arm;
		KArmControl* ptr;
		ros::NodeHandle nh;
	public:
		pub_sub(ros::NodeHandle& n){
			nh = n;
			pub_ = nh.advertise<std_msgs::Int32>("arm_start",10);
			sub_arm = nh.subscribe<std_msgs::Int32>("arm_complete", 10, &pub_sub::callback, this);
			ptr = new KArmControl(nh);
			ROS_INFO("here");
		}

		void callback(const std_msgs::Int32::ConstPtr& begin){
			ROS_INFO("here1");
			ptr->set_pose_goal(0,-0.5,0.1,1,0,0,1);
			ptr->plan_goal();
			ptr->set_pose_goal(0,-0.7,0.1,1,0,0,1);
			ptr->plan_goal();
			ptr->grasp_control(3300);	
			ptr->set_pose_goal(.08,-0.7,0.15,1,0,0,1);
			ptr->plan_goal();
			ptr->grasp_control(0);	
			ptr->set_joint_goal(4.712,2.845,0,0.750,4.625,4.486,4.887);
			ptr->plan_goal();
			std_msgs::Int32 reply;
			reply.data = 1;
			pub_.publish(reply);	
		}




};

*/


int main(int argc, char** argv){
	ros::init(argc, argv, "kinova_arm_planning");
	ros::NodeHandle node_handle;  
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::Publisher pub = node_handle.advertise<std_msgs::Int32>("arm_complete",10);

    //pub_sub trial1(node_handle);

	KArmControl demo(node_handle);
	//demo1.set_pose_goal(0.54882, -0.30854,  0.658410, .68463, -0.22436, 0.68808, 0.086576);
	//demo1.plan_goal();
	//demo1.set_pose_goal(0.74882, -0.30854,  0.658410, .68463, -0.22436, 0.68808, 0.086576);
	//demo1.plan_goal();
	//demo1.set_joint_goal(0,0,0,0,0,-1.0);
	//demo1.plan_goal();
	//demo1.add_object();
	//demo1.pick_and_place();
	//demo1.grasp_control(6400);
				demo.add_object();
				demo.grasp_control(1); 
				demo.set_pose_goal(0,-0.5,.5,1,0,0,1);
				demo.plan_goal();
				demo.set_pose_goal(0,-0.5,.3,1,0,0,1);
				demo.plan_goal();
				demo.set_pose_goal(0,-0.5,.2,1,0,0,1);
				demo.plan_goal();
				demo.set_pose_goal(0,-0.5,.09,1,0,0,1);
				demo.plan_goal();
			    demo.set_pose_goal(0,-0.72,.06,1,0,0,1);
			    demo.plan_goal();
   				demo.grasp_control(5900);  
			    demo.set_pose_goal(.15,-0.78,.13,1,0,0,1);
			    demo.plan_goal(); 
			    demo.set_pose_goal(.22,-0.78,.06,1,0,0,1);
			    demo.plan_goal(); 
			    demo.grasp_control(1);
				system("rosservice call /'j2s7s300_driver'/in/home_arm");
			    //demo.set_joint_goal(4.712,2.845,0,0.750,4.625,4.486,4.887);
			    //demo.plan_goal();

/*	demo1.set_pose_goal(.04,-0.7,0.035,1,0,0,1);
	demo1.plan_goal();
	demo1.grasp_control(6000);	
	demo1.set_pose_goal(.2,-0.7,0.075,1,0,0,1);
	demo1.plan_goal();
	demo1.grasp_control(1);	*/
	//demo1.set_pose_goal(.212,-.265,.5047,.646,.318,.423,.55);
	//demo1.set_pose_goal(0,-0.3,0.5,1,0,0,1);
	//demo1.plan_goal();
	//std_msgs::Int32 reply;
	//reply.data = 1;
	//pub.publish(reply);	
} 
