#include "bayes_filter/ParticleFilter.h"
#include <random>


ParticleFilter::ParticleFilter(nav_msgs::OccupancyGrid grid):
	occupancy_map_(&grid),
	map_width(occupancy_map_->info.width),
	map_height(occupancy_map_->info.height),
	map_resolution(occupancy_map_->info.resolution)
{
	prev_pose.x = 0;
	prev_pose.y = 0;
	prev_pose.theta = 0;

	Particles p;
	for(int i = 0; i < num_particle; i++){
		p.x = randGen() * 16 - 16 / 2.0;
		p.y = randGen() * 16 - 16 / 2.0;
		p.yaw = randGen() * 2 * 3.14 - 3.14;
		p.weight = 1;
		particle_set.push_back(p);
	}


}

inline double ParticleFilter::randGen()
{
	return (double)(rand () % 10000) / 10000.0;
}

geometry_msgs::PoseArray ParticleFilter::localization(geometry_msgs::Pose2D current_pose)
{

	geometry_msgs::PoseArray particles_msg;
	particles_msg.header.stamp = ros::Time::now();
	particles_msg.header.frame_id = "/odom";

	if ((abs(current_pose.x - prev_pose.x) < 0.00001) &&
		(abs(current_pose.y - current_pose.y) < 0.00001) &&
		(abs(current_pose.theta - prev_pose.theta) < 0.001))
	{
		prev_pose = current_pose;
		// return particles_msg;
	}
	else{
		model.motion(current_pose, prev_pose, particle_set);

	}


	tf2::Quaternion myQuaternion;
	// std::cout << " size : " << particle_set.size() << std::endl;
	for(auto &sample:particle_set) {
		geometry_msgs::Pose pose;
		// double x_map = (sample.x * occupancy_map_->info.resolution) + occupancy_map_->info.origin.position.x + occupancy_map_->info.resolution/2;
		// double y_map = (sample.y * occupancy_map_->info.resolution) + occupancy_map_->info.origin.position.y + occupancy_map_->info.resolution/2;

		myQuaternion.setRPY( 0, 0, sample.yaw);
		// pose.position.x = x_map;
		// pose.position.y = y_map;
		pose.position.x = sample.x;
		pose.position.y = sample.y;
		pose.position.z = 0;

		pose.orientation.x = myQuaternion.getX();
		pose.orientation.y = myQuaternion.getY();
		pose.orientation.z = myQuaternion.getZ();
		pose.orientation.w = myQuaternion.getW();

		particles_msg.poses.insert(particles_msg.poses.begin(), pose);
	}



	prev_pose = current_pose;


	return particles_msg;
}
