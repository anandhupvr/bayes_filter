#include <stdio.h>
#include <ros/ros.h>
#include "bayes_filter/ParticleFilter.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
#include "nav_msgs/Odometry.h"
#include <nav_msgs/OccupancyGrid.h>

ros::Publisher vis_pub;

nav_msgs::OccupancyGrid map_;

geometry_msgs::Pose2D loc;
// geometry_msgs::Pose2D current;
void odomCallBack(const nav_msgs::Odometry& msg){
	loc.x = msg.pose.pose.position.x;
	loc.y = msg.pose.pose.position.y;
	loc.theta = tf::getYaw(msg.pose.pose.orientation);
}

void mapCallBack(const nav_msgs::OccupancyGrid msg){
	map_ = msg;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "filter");
	ros::NodeHandle nh;
	ros::Publisher particle_pub;

	particle_pub = nh.advertise<geometry_msgs::PoseArray>("/particle_cloud",1);

	ros::Subscriber map_sub = nh.subscribe("/map2d", 1, mapCallBack);

	ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odomCallBack);


	ParticleFilter *pf = new ParticleFilter(map_);
	while(ros::ok())
	{
		geometry_msgs::PoseArray particle_msgs = pf->localization(loc);

		particle_pub.publish(particle_msgs);
		ros::spinOnce();
		
	}

	return 0;

}