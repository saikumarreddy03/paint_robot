#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <Planar2R.hpp>
	/* Write your code her for publishing to /pubJointStates topic
	** The message type is sensor_msgs/JointState 
	** The name field should be an array of names of all four joints
	** The header.stamp field should be ros::Time::now() 
	** The position field should be an array of double values
	** Keep filling the values inside the while(ros::ok()) loop
	** Elapsed time can be calculated as:
	** ros::Time start = ros::Time::now();
	** double diff = (ros::Time::now() - start).toSec();
	** Make the values sinusodial depending on variable diff or anything you like
	** Publish the msg 
	** The lines to be changed or added are marked*/
int main(int argc, char **argv)
{
	IRlibrary::Planar2R planar_obj;
	IRlibrary::Vec2 temp_vector;
	IRlibrary::Vec2 q;
	temp_vector << 2,-1;
	ros::init(argc, argv, "genConfig");
	ros::NodeHandle n;
	ros::Rate loop_rate(30);
	
	ros::Duration(0.01).sleep();
	ros::Publisher configPub;
	configPub = n.advertise <sensor_msgs::JointState> ("/pubJointStates", 5); /* Fix this line. Do NOT change "/pubJointStates" */
	ros::Time start = ros::Time::now();
	sensor_msgs::JointState new_state;
	new_state.name = {"joint1", "joint2"};
	new_state.header.stamp = ros::Time::now();
	double diff = (ros::Time::now() - start).toSec();
	new_state.position = { M_PI * cos(diff), M_PI * sin(diff)};
	
	
	//practice
	planar_obj.setXY(temp_vector);
	q = planar_obj.getConfig();
	new_state.position[0] = q[0];
	new_state.position[1] = q[1];
	configPub.publish(new_state);
	
	
	while (ros::ok())
	{
		diff = (ros::Time::now() - start).toSec();
		new_state.header.stamp = ros::Time::now();
		new_state.position[0] = M_PI*cos(ros::Time::now().toSec());
		new_state.position[1] = M_PI*cos(ros::Time::now().toSec());
		configPub.publish(new_state);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
