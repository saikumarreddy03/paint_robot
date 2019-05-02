#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <Planar2R.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <Spatial3R.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "genConfig");
	ros::NodeHandle n;
	ros::Rate loop_rate(30);

	ros::Duration(0.01).sleep();
	ros::Publisher configPub;
	configPub = n.advertise <sensor_msgs::JointState> ("/pubJointStates", 5); /* Fix this line. Do NOT change "/pubJointStates" */
	ros::Time start = ros::Time::now();
	sensor_msgs::JointState new_state;
	new_state.name = {"joint1", "joint2","joint3"};
	new_state.header.stamp = ros::Time::now();
	double diff = (ros::Time::now() - start).toSec();
	new_state.position = { 0, 0, 0 };

	double l1, l2, l3;
	n.getParam("link_lengths/l1", l1);
	n.getParam("link_lengths/l2", l2);
	n.getParam("link_lengths/l3", l3);

	double objectList[2][3] = {{1.,2,3.},{4.,5.,6.}};
	IRlibrary::Vec3 object;
	IRlibrary::Vec3 placementLine;

	while (ros::ok()){
	for(int i=0; i<3 && ros::ok();i++){
		IRlibrary::Spatial3R obj3r;
		obj3r.setLinks(l1, l2, l3);
		auto q = obj3r.getConfig();
		object << objectList[i][0], objectList[i][1], objectList[i][2];
		obj3r.setX(object);
		q = obj3r.getConfig();
		new_state.position[0] = q[0];
		new_state.position[1] = q[1];
		new_state.position[2] = q[2];
		configPub.publish(new_state); ros::Duration(0.5).sleep();
	}
	}
	return 0;
}
