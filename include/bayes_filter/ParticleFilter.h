#include <iostream>
#include <ros/ros.h>
#include "bayes_filter/ParticleType.h"
// #include "bayes_filter/Particle.h"
#include "bayes_filter/Map.h"
#include "bayes_filter/Models.h"
#include <geometry_msgs/PoseArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Pose2D.h>

class ParticleFilter{

	public:
		nav_msgs::OccupancyGrid* occupancy_map_;
		int map_width;
		int map_height;
		float map_resolution;

		Models model;
		int num_particle = 500;
		std::vector<Particles> particle_set;
		
		geometry_msgs::Pose2D prev_pose;
		ParticleFilter(nav_msgs::OccupancyGrid grid);
		// ParticleFilter();
		inline double randGen();
		geometry_msgs::PoseArray localization(geometry_msgs::Pose2D pose);

};