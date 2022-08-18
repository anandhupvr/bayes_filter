#include "bayes_filter/Map.h"


Grid::Grid(ros::NodeHandle *nh){

	// map_sub_ = nh->subscribe("/map", 1, &Grid::mapCallback, this);
	// map_client = nh->serviceClient<nav_msgs::GetMap>("/map");
	ros::ServiceClient mapClient = nh->serviceClient<nav_msgs::GetMap>("static_map");

	printf("subscribed");
}

void Grid::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	// occupancy_map_->header = msg->header;
	// occupancy_map_->info = msg->info;
	// occupancy_map_->data = msg->data;
}

void Grid::showMapInfo()
{
	printf("show info node");
	// printf("%s",occupancy_map_->header.frame_id);
	// std::cout << " map_width = " << occupancy_map_->info.width << "  , map_height = " << occupancy_map_->info.height << std::endl;
	// printf("map_width = %d , map_height = %d \n",occupancy_map_->info.width, occupancy_map_->info.height );
}

void Grid::getMap()
{
	// ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>("/static_map");
	nav_msgs::GetMap srv;
	map_client.call(srv);
	occupancy_map_ = srv.response.map;
	printf("map \n");
	printf("%lu \n" , occupancy_map_.info.width );
}