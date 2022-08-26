#include <stdio.h>
#include <ros/ros.h>
#include "bayes_filter/ParticleFilter.h"
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
#include "nav_msgs/Odometry.h"
#include <nav_msgs/OccupancyGrid.h>
// #include <sensor_msgs/LaserScan.h>

#include<thread>
#include<chrono>

ros::Publisher vis_pub;

nav_msgs::OccupancyGrid map_;

std::list<float>* scans;
sensor_msgs::LaserScan scan_data;

geometry_msgs::Pose2D loc;
// geometry_msgs::Pose2D current;
void odomCallBack(const nav_msgs::Odometry& msg){
	loc.x = msg.pose.pose.position.x;
	loc.y = msg.pose.pose.position.y;
	loc.theta = tf::getYaw(msg.pose.pose.orientation);
}

std::list<int> laserRanges(){
	std::list<int> ranges;
	int val = 320;
	for(int i = 0; i < 320; i++){
		ranges.push_back(val);
		val += 2;
	}

	return ranges;

}

void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr msg){
	map_ = *msg;
	std::cout << "inside mapCallBack map : " << map_.info.width << " , " << map_.info.height << std::endl;


	// map_.resolution = msg.info.resolution;
	// map_.xWidth = msg.info.width;
	// map_.yWidth = msg.info.height;
	// map_.xMin = 0;
	// map_.yMin = 0;
	// map_.xMax = map_.xWidth;
	// map_.yMax = map_.yWidth;

	// // int i = 0;
	// for(int y=0; y<map_.yMax; y++){
	// 	std::vector<int> row;
	// 	for(int x=0; x<map_.xMax; x++){
	// 		row.push_back((int)msg.data[i]);
	// 		// if((int)msg.data[i]==100)
	// 		// 	occupied_.push_back(std::make_pair(y,x));
	// 		// else if((int)msg.data[i]==0)
	// 		// 	free_.push_back(std::make_pair(y,x));
	// 		// i++;
	// 	}
	// 	map_.data.push_back(row);
	// }



}


void laserScanMsgCallBack(const sensor_msgs::LaserScan msg){
	scan_data = msg;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "filter");
	ros::NodeHandle nh;
	ros::Publisher particle_pub;

	particle_pub = nh.advertise<geometry_msgs::PoseArray>("/particle_cloud",1);

	// ros::Subscriber map_sub = nh.subscribe("/map2d", 1, mapCallBack);

	ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odomCallBack);

	ros::Subscriber laser_scan_sub_  = nh.subscribe("scan", 10, laserScanMsgCallBack);

	ros::Rate loop(10);

	std::chrono::system_clock::time_point waitUntil = std::chrono::system_clock::now() + std::chrono::seconds(2);
	std::this_thread::sleep_until(waitUntil);

	ParticleFilter *pf = new ParticleFilter(nh);

	while(ros::ok())
	{
		ros::spinOnce();
		if(pf->occupancy_map_.info.width == 0) continue;
		geometry_msgs::PoseArray particle_msgs = pf->localization(loc, scan_data);

		// particle_pub.publish(particle_msgs);
		loop.sleep();
	}

	return 0;

}