#include <stdio.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>


class Grid{

	public:
		int width;
		int height;
		float resolution;
		// ros::NodeHandle nh_;
		nav_msgs::OccupancyGrid occupancy_map_;
		Grid(ros::NodeHandle *nh);
		void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
		void showMapInfo();
		void getMap();
	private:
		ros::Subscriber map_sub_;
		ros::ServiceClient map_client;



};