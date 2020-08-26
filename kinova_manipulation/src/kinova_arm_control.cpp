#include "kinova_manipulation/kinova_arm_control.hpp"

KArmControl::KArmControl(ros::NodeHandle& node_handle){
	nh = node_handle;	
	nh.param<bool>("/robot_connected",robot_connected_,true);
	display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 1, true);
	move_group_ptr = MGP(new moveit::planning_interface::MoveGroupInterface("arm"));
	finger_group_ptr = MGP(new moveit::planning_interface::MoveGroupInterface("gripper"));
	planning_scene_ptr = PSP(new moveit::planning_interface::PlanningSceneInterface());
	finger_action_client_ptr = FPA(new actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>("/j2s7s300_driver/fingers_action/finger_positions", false));
	move_group_ptr->setMaxVelocityScalingFactor(0.5);
	move_group_ptr->setPlanningTime(5.0);
	move_group_ptr->setPoseReferenceFrame("world");
	move_group_ptr->setEndEffectorLink("j2s7s300_end_effector");

robot_model_ = rm_loader_.getModel();
  if (!robot_model_)
  {
    ROS_ERROR("Could not load robot description");
  }

  // create a RobotState to keep track of the current robot pose
  robot_state_.reset(new moveit::core::RobotState(robot_model_));
  if (!robot_state_)
  {
    ROS_ERROR("Could not get RobotState from Model");
  }
  robot_state_->setToDefaultValues();


    while(robot_connected_ && !finger_action_client_ptr->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the finger action server to come up");
    }
}

void KArmControl::set_pose_goal(float vx, float vy, float vz, float w_val, float x_val, float y_val, float z_val){
	tf::Pose new_pose;
	new_pose.setOrigin(tf::Vector3(vx, vy, vz));
	new_pose.setRotation(tf::Quaternion(w_val, x_val, y_val, z_val));
	geometry_msgs::Pose target_pose1;
	tf::poseTFToMsg(new_pose, target_pose1);
	move_group_ptr->setPoseTarget(target_pose1);
	
	ROS_INFO("Pose Goal set to origin x: %f y: %f z: %f",  vx, vy, vz);
	ROS_INFO("Pose Goal set to rotation w: %f x: %f y: %f z: %f", w_val, x_val, y_val, z_val);

}


void KArmControl::set_joint_goal(double j1, double j2, double j3, double j4, double j5, double j6, double j7){
	std::vector<double> group_variable_values; move_group_ptr->getCurrentState()->copyJointGroupPositions(move_group_ptr->getCurrentState()->getRobotModel()->getJointModelGroup(move_group_ptr->getName()), group_variable_values); 
	group_variable_values[0] = j1;
	group_variable_values[1] = j2; 
	group_variable_values[2] = j3;
	group_variable_values[3] = j4;
	group_variable_values[4] = j5;
	group_variable_values[5] = j6;
	group_variable_values[6] = j7;

	move_group_ptr->setJointValueTarget(group_variable_values);

}


void KArmControl::add_object(){
	
	collision_object.header.frame_id = "world";
	collision_object.id = "box1";
	
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

	std::vector<moveit_msgs::CollisionObject> collision_objects;  
	collision_objects.push_back(collision_object);  

	ROS_INFO("Added an object into the world");  
	planning_scene_ptr->applyCollisionObjects(collision_objects);
	sleep(2.0);
	
}


void KArmControl::plan_goal(){
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success = (move_group_ptr->plan(my_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS);

	ROS_INFO("Visualizing plan (pose goal) %s",success?"":"FAILED");    
	//sleep(5.0);
/*
	if (success == true){
		ROS_INFO("Visualizing plan (again)");    
		display_trajectory.trajectory_start = my_plan.start_state_;
		display_trajectory.trajectory.push_back(my_plan.trajectory_);
		display_publisher.publish(display_trajectory);
		sleep(5.0);
	}*/
	move_group_ptr->move();

}


bool KArmControl::grasp_control(float finger_position){
    if(robot_connected_ == false)
    {
        if (finger_position>0.5*6400.0)
        {
          finger_group_ptr->setNamedTarget("Close");
		  ROS_INFO("CLOSED GRIPPER");
        }
        else
        {
          finger_group_ptr->setNamedTarget("Open");
		  ROS_INFO("OPENED GRIPPER");
        }
        finger_group_ptr->move();
        return true;
    }

    if (finger_position < 0)
    {
        finger_position = 0.0;
    }
    else
    {
		float max_pos = 6400;
        finger_position = std::min(finger_position, max_pos);
    }

    kinova_msgs::SetFingersPositionGoal goal;
    goal.fingers.finger1 = finger_position;
    goal.fingers.finger2 = goal.fingers.finger1;
    goal.fingers.finger3 = goal.fingers.finger1;
    finger_action_client_ptr->sendGoal(goal);

    if (finger_action_client_ptr->waitForResult(ros::Duration(5.0)))
    {
        finger_action_client_ptr->getResult();
        return true;
    }
    else
    {
        finger_action_client_ptr->cancelAllGoals();
        ROS_WARN_STREAM("The gripper action timed-out");
        return false;
    }	



}

void KArmControl::pick_and_place(){
	
	set_joint_goal(4.712, 4.224, 0, 1.676, 4.625, 3.107, 4.887);
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success = (move_group_ptr->plan(my_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS);

	ROS_INFO("Visualizing plan %s",success?"":"FAILED");
	sleep(10.0);
	move_group_ptr->setMaxVelocityScalingFactor(0.5);
	move_group_ptr->move();
	
	ROS_INFO("Attach the object to the robot");  
	move_group_ptr->attachObject(collision_object.id);  
	sleep(4.0);
	grasp_control(6400);
	
	set_joint_goal(3.05, 4.05, 0, 1.676, 4.625, 3.107, 4.887);
	success = (move_group_ptr->plan(my_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS);
	ROS_INFO("Visualizing plan %s",success?"":"FAILED");
	sleep(10.0);
	move_group_ptr->setMaxVelocityScalingFactor(0.5);
	move_group_ptr->move();



	ROS_INFO("Detach the object from the robot");  
	move_group_ptr->detachObject(collision_object.id);  
	sleep(4.0);
	grasp_control(0);

	ROS_INFO("Remove the object from the world");  
	std::vector<std::string> object_ids;
	object_ids.push_back(collision_object.id);  
	planning_scene_ptr->removeCollisionObjects(object_ids);
	sleep(4.0);

	move_group_ptr->setPlanningTime(5.0);

}
