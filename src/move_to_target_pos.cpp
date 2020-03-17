#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>




class CranePlusArmController {
	
public:
	CranePlusArmController() {
	
		sub = nh.subscribe("robot_arm_move_position", 1,&CranePlusArmController::positionCallback, this);
		
		SetInitialPose();

	}

private:
	ros::NodeHandle nh;
	ros::Subscriber sub;
	moveit::planning_interface::MoveGroupInterface arm{"arm"};

	geometry_msgs::PoseStamped arm_pose;

	void SetInitialPose() {
		arm.setPoseReferenceFrame("base_link");
		arm.setNamedTarget("vertical");	


		arm_pose.header.frame_id = "base_link";
		arm_pose.pose.position.x = 0.18;
		arm_pose.pose.position.y = 0.0;
		arm_pose.pose.position.z = 0.10;
		arm_pose.pose.orientation.x = 0.0;
		arm_pose.pose.orientation.y = 0.707106;
		arm_pose.pose.orientation.z = 0.0;
		arm_pose.pose.orientation.w = 0.707106;

		arm.asyncMove();
	}

	void positionCallback(const geometry_msgs::Pose& msg) {
		ROS_INFO("pos");

		//arm.setNamedTarget("vertical");

		/*	
		arm_pose.pose.position.x += 0.01;
                arm_pose.pose.position.y += 0.01;
                arm_pose.pose.position.z += 0.01;

		arm_pose.header.frame_id = "base_link";
		*/
		
		arm_pose.header.frame_id = "base_link";
		arm_pose.pose.position.x = msg.position.x;
		arm_pose.pose.position.y = msg.position.y;
		arm_pose.pose.position.z = msg.position.z;
		arm_pose.pose.orientation.x = msg.orientation.x;
		arm_pose.pose.orientation.y = msg.orientation.y;
		arm_pose.pose.orientation.z = msg.orientation.z;
		arm_pose.pose.orientation.w = msg.orientation.w;
		


		ROS_INFO("[MOVEIT]Move Arm_position[%f, %f, %f]", arm_pose.pose.position.x, arm_pose.pose.position.y, arm_pose.pose.position.z);

		arm.setPoseReferenceFrame("base_link");
		arm.setPoseTarget(arm_pose);
		arm.setGoalTolerance(0.02);

		//arm.asyncMove();
		
  		if (!arm.asyncMove()) {
    			ROS_WARN("Could not move to prepare pose");
  		}
		
		ROS_INFO("SUCCESS?");
	}

};
/*
void positionCallback(const geometry_msgs::PoseStamped& msg) {
    ROS_INFO("position");

}
*/

int main(int argc, char **argv) {
  ros::init(argc, argv, "crane_plus_move_arm");
  CranePlusArmController armController;
  ros::spin();
  //ros::AsyncSpinner spinner(2);
  //spinner.start();
  return 0;
  //ros::NodeHandle nh;


  //ros::Subscriber sub = nh.subscribe("robot_arm_move_position", 1, positionCallback);


  //ros::AsyncSpinner spinner(2);
  //spinner.start();


  //moveit::planning_interface::MoveGroupInterface arm("arm");
  //arm.setPoseReferenceFrame("base_link");


  //actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper(
  //    "/crane_plus_gripper/gripper_command",
  //    "true");
  //gripper.waitForServer();


  //ros::spin();
  //return 0;


  // Prepare
  /*
  ROS_INFO("Moving to prepare pose");
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "base_link";
  pose.pose.position.x = 0.2;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.1;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.707106;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 0.707106;


  arm.setPoseTarget(pose);
  if (!arm.move()) {
    ROS_WARN("Could not move to prepare pose");
    return 1;
  }
*/

  /*
  ROS_INFO("Closing gripper");
  control_msgs::GripperCommandGoal goal;
  goal.command.position = 0;
  gripper.sendGoal(goal);
  bool finishedBeforeTimeout = gripper.waitForResult(ros::Duration(30));
  if (!finishedBeforeTimeout) {
    ROS_WARN("Gripper close action did not complete");
    return 1;
  }


  ros::shutdown();
  return 0;
*/
}
