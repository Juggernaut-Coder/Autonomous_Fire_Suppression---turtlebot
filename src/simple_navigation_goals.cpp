#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <string.h>
#include <std_msgs/Int32.h>
#include <sstream>
#include <tf/tf.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"



typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
MoveBaseClient;

int main(int argc, char** argv){
	int no_w;
	int tmp;
	int ret_scan;
	printf("Enter the number of waypoints you wish to input:");
	scanf("%d",&no_w);
	float xyz[no_w][3];
	// collecting input
	for(int j=0;j<no_w;j++) {
		printf("enter value x, y, z of %d waypoint: ",j+1);
		ret_scan = scanf("%f %f %f",&xyz[j][0],&xyz[j][1],&xyz[j][2]);
		while(ret_scan!=3) {
			while((tmp=getchar()) != EOF && tmp != '\n');
			printf("Incorrect type of input\n");
			printf("enter value x, y, z of %d waypoint: ",j+1);
			ret_scan = scanf("%f %f %f",&xyz[j][0],&xyz[j][1],&xyz[j][2]);
		}
	}
	//_________________________X___________________________
	//ros initialization
 	ros::init(argc, argv, "simple_navigation_goals");
	ros::NodeHandle nh;
	ros::Publisher chatter_pub = nh.advertise<std_msgs::Int32>("blink", 6);
	ros::Rate loop_rate(0.5);
 	//tell the action client that we want to spin a thread by default
 	MoveBaseClient ac("move_base", true);
 	//wait for the action server to come up
 	while(!ac.waitForServer(ros::Duration(5.0))){
 		ROS_INFO("Waiting for the move_base action server to come up");
 	}
 	move_base_msgs::MoveBaseGoal goal;
 	//______________________________X_____________________________________
	// Set orientation.w
	goal.target_pose.pose.orientation.w = 1;
	// __________for loop executing waypoints
	for(int j=0;j<no_w;j++) {
		goal.target_pose.header.frame_id = "map";
 		goal.target_pose.header.stamp = ros::Time::now();
 		goal.target_pose.pose.position.x = xyz[j][0]; 
 		goal.target_pose.pose.position.y = xyz[j][1];
		goal.target_pose.pose.orientation.z = xyz[j][2];
		
		ROS_INFO("Sending goal %d",j+1);
	 	ac.sendGoal(goal);
	 	ac.waitForResult();
		if(ros::ok()) {
			loop_rate.sleep();
			std_msgs:: Int32 msg;
			msg.data = 15;
			ROS_INFO("%d", msg.data);
			chatter_pub.publish(msg);
			ros::spinOnce();
			loop_rate.sleep();
		}
	
	}
	
 	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
 		ROS_INFO("Success");
  	} else {
 		ROS_INFO("Failure");
	}
 	return 0;
}

