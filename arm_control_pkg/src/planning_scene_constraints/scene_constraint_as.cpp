#include <boost/filesystem.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h> 
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <arm_control_pkg/SceneConstraintAction.h>

typedef boost::shared_ptr<moveit::planning_interface::PlanningSceneInterface> PSP;

class PlanningSceneConstraintAS{
	protected:	
		//variables for nodehandle, action server, feedback and results
		ros::NodeHandle nh_;
		actionlib::SimpleActionServer<arm_control_pkg::SceneConstraintAction> as_;
		std::string action_name_;
		arm_control_pkg::SceneConstraintFeedback feedback_;
	    arm_control_pkg::SceneConstraintResult result_;

		moveit_msgs::CollisionObject collision_object;
		PSP planning_scene_ptr;
		std::vector<moveit_msgs::CollisionObject> constant_collision_objects;  
		std::vector<moveit_msgs::CollisionObject> temporary_collision_objects;  

	public:
		//Starts action server with name given in main function and begins async spinner for moveit
		PlanningSceneConstraintAS(std::string name):
		as_(nh_, name, boost::bind(&PlanningSceneConstraintAS::executeCB, this, _1), false)
		, action_name_(name){
			as_.start();
			ros::AsyncSpinner spinner(1);
			spinner.start();
			planning_scene_ptr = PSP(new moveit::planning_interface::PlanningSceneInterface());
	
		}

		void executeCB(const arm_control_pkg::SceneConstraintGoalConstPtr &goal){
			ROS_INFO("POSITION CONSTRAINT ACTION EXECUTING");
			
			//checks for preempt
		 	if (as_.isPreemptRequested() || !ros::ok()){
        		ROS_INFO("%s: Preempted", action_name_.c_str());
        		as_.setPreempted();
      		}

			//if stage start is true execute positioning of arm if it completes succesfully return true as result
			//Needs error checking
			if(goal->constraint_request == "initialize"){
				initialize_scene();	

				ROS_INFO("SUCCESS");
				result_.constraint_result = true;
				as_.setSucceeded(result_);
			}
			else if(goal->constraint_request == "manipulation"){
				manipulation_constraints();

				ROS_INFO("SUCCESS");
				result_.constraint_result = true;
				as_.setSucceeded(result_);
			}
			else if(goal->constraint_request == "remove_temp"){
				remove_temp_objects();

				ROS_INFO("SUCCESS");
				result_.constraint_result = true;
				as_.setSucceeded(result_);
			}
			else if(goal->constraint_request == "remove_constant"){
				remove_constant_objects();

				ROS_INFO("SUCCESS");
				result_.constraint_result = true;
				as_.setSucceeded(result_);
			}else{
				ROS_INFO("FAILED");
				result_.constraint_result = false;
				as_.setSucceeded(result_);
			}
		}

		void initialize_scene(){
			collision_object.header.frame_id = "world";
			collision_object.id = "base";
			
			shape_msgs::SolidPrimitive primitive;
			primitive.type = primitive.BOX;
			primitive.dimensions.resize(3);
			primitive.dimensions[0] = 2;
			primitive.dimensions[1] = 2;
			primitive.dimensions[2] = 0.02;

			geometry_msgs::Pose box_pose;
			box_pose.orientation.w = 1.0;
			box_pose.position.x = 0;
			box_pose.position.y = 0;
			box_pose.position.z = -.01;

			collision_object.primitives.push_back(primitive);
			collision_object.primitive_poses.push_back(box_pose);
			collision_object.operation = collision_object.ADD;

			constant_collision_objects.push_back(collision_object);  

			ROS_INFO("Added constant objects into the world");  
			planning_scene_ptr->applyCollisionObjects(constant_collision_objects);
			sleep(2.0);	
		}
		
		void manipulation_constraints(){
			collision_object.header.frame_id = "world";
			collision_object.id = "left_side";
			
			shape_msgs::SolidPrimitive primitive;
			primitive.type = primitive.BOX;
			primitive.dimensions.resize(3);
			primitive.dimensions[0] = .02;
			primitive.dimensions[1] = 1;
			primitive.dimensions[2] = 1;

			geometry_msgs::Pose box_pose;
			box_pose.orientation.w = 1.0;
			box_pose.position.x = .5;
			box_pose.position.y = -.5;
			box_pose.position.z = .5;

			collision_object.primitives.push_back(primitive);
			collision_object.primitive_poses.push_back(box_pose);
			collision_object.operation = collision_object.ADD;
			temporary_collision_objects.push_back(collision_object);  

			collision_object.header.frame_id = "world";
			collision_object.id = "right_side";
			
			primitive.type = primitive.BOX;
			primitive.dimensions.resize(3);
			primitive.dimensions[0] = .02;
			primitive.dimensions[1] = 1;
			primitive.dimensions[2] = 1;

			box_pose.orientation.w = 1.0;
			box_pose.position.x = -.5;
			box_pose.position.y = -.5;
			box_pose.position.z = .5;

			collision_object.primitives.push_back(primitive);
			collision_object.primitive_poses.push_back(box_pose);
			collision_object.operation = collision_object.ADD;
			temporary_collision_objects.push_back(collision_object);  

			collision_object.header.frame_id = "world";
			collision_object.id = "back_side";
			
			primitive.type = primitive.BOX;
			primitive.dimensions.resize(3);
			primitive.dimensions[0] = 1;
			primitive.dimensions[1] = .02;
			primitive.dimensions[2] = 1;

			box_pose.orientation.w = 1.0;
			box_pose.position.x = 0;
			box_pose.position.y = -1;
			box_pose.position.z = .5;

			collision_object.primitives.push_back(primitive);
			collision_object.primitive_poses.push_back(box_pose);
			collision_object.operation = collision_object.ADD;
			temporary_collision_objects.push_back(collision_object);  

			collision_object.header.frame_id = "world";
			collision_object.id = "top_side";
			
			primitive.type = primitive.BOX;
			primitive.dimensions.resize(3);
			primitive.dimensions[0] = 1;
			primitive.dimensions[1] = 1;
			primitive.dimensions[2] = .02;

			box_pose.orientation.w = 1.0;
			box_pose.position.x = 0;
			box_pose.position.y = -.5;
			box_pose.position.z = .99;

			collision_object.primitives.push_back(primitive);
			collision_object.primitive_poses.push_back(box_pose);
			collision_object.operation = collision_object.ADD;
			temporary_collision_objects.push_back(collision_object);  


			ROS_INFO("Added an temporary objects into the world");  
			planning_scene_ptr->applyCollisionObjects(temporary_collision_objects);
			sleep(2.0);
			
		}

		void remove_temp_objects(){
					
			ROS_INFO("Remove the temporary objects from the world");  
			std::vector<std::string> object_ids;
			
			for(auto &object : temporary_collision_objects){
				object_ids.push_back(object.id);  	
			}
			planning_scene_ptr->removeCollisionObjects(object_ids);
			sleep(4.0);
		}

		void remove_constant_objects(){
					
			ROS_INFO("Remove the constant objects from the world");  
			std::vector<std::string> object_ids;
			
			for(auto &object : constant_collision_objects){
				object_ids.push_back(object.id);  	
			}
			planning_scene_ptr->removeCollisionObjects(object_ids);
			sleep(4.0);
		}

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "scene_constraint_as");

  PlanningSceneConstraintAS demo("scene_constraint_as");
  ros::spin();

  return 0;
}
