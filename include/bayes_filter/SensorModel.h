#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <list>
#include "bayes_filter/ParticleType.h"
#include <sensor_msgs/LaserScan.h>
#include <cmath>

class SensorModel{

public:
	nav_msgs::OccupancyGrid map;


	double _sensorOffsetx = 0.0; // -0.074
	double _sensorOffsety = 0.0;
	double _pi = 3.14159;
	double _zhit = 0.8;

	std::vector<std::pair<int,int>> occupied_;
	// SensorModel(nav_msgs::OccupancyGrid map_);
	SensorModel();
	void mapUpdate(nav_msgs::OccupancyGrid map_);
	std::vector<std::pair<int,int>>  _find_objects(nav_msgs::OccupancyGrid map);
	double _findMinDistance(double x, double y);
	double gaussianDistribution(double mean, double standardDeviation, double x);
	void measurementModel(std::vector<Particles> &samples, sensor_msgs::LaserScan scan, nav_msgs::OccupancyGrid map);

};