#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Pose2D.h>
#include <cmath>
#include <random>
#include "bayes_filter/ParticleType.h"

class MotionModel{
	public:
		// Xt pose
		geometry_msgs::Pose2D latest_pose;
		// Xt-1 pose
		geometry_msgs::Pose2D prev_pose;
		// Xt - Xt-1
		geometry_msgs::Pose2D delta_pose;

		MotionModel();
		void odom(geometry_msgs::Pose2D latest_pose, geometry_msgs::Pose2D prev_pose, std::vector<Particles> &samples);
		void gaussNoise();
		float SampleStandardNormalDistribution(float var);

};