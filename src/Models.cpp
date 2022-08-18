#include "bayes_filter/Models.h"

Models::Models()
{}

// void Models::getDistance()
// {

// }

float Models::SampleStandardNormalDistribution(float var)
{
	float sum = 0;
	for (int i = 0;i < 12; i++)
		//LO + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(HI-LO)))
		sum += (rand() - RAND_MAX / 2) / (float)RAND_MAX * 2;
	return (var / 6.0) * sum;
}


void Models::motion(geometry_msgs::Pose2D latest_pose, geometry_msgs::Pose2D prev_pose, std::vector<Particles> &samples){

	double delta_trans = sqrt(pow((latest_pose.x - prev_pose.x), 2) + pow((latest_pose.y - prev_pose.y), 2));
	double deltarot1 = atan2(latest_pose.y - prev_pose.y, latest_pose.x - prev_pose.x) - prev_pose.theta;
	double deltarot2 = latest_pose.theta - prev_pose.theta - deltarot1;
	// float alpha1_ = 0.025;
	// float alpha2_ = 0.025;
	// float alpha3_ = 0.4;
	// float alpha4_ = 0.4;
	// float deltarot1_hat = deltarot1 - SampleStandardNormalDistribution(alpha1_*deltarot1 + alpha2_*delta_trans);
	// float deltatrans1_hat  = delta_trans - SampleStandardNormalDistribution(alpha3_*delta_trans + alpha4_*(deltarot1 + deltarot2));
	// float deltarot2_hat  = deltarot2 - SampleStandardNormalDistribution(alpha1_*deltarot2 + alpha2_*delta_trans);
	for(auto &sample:samples){
		sample.x = sample.x + (delta_trans * cos(sample.yaw + deltarot1));
		sample.y = sample.y + (delta_trans * sin(sample.yaw + deltarot1));
		sample.yaw = (sample.yaw + deltarot1 + deltarot2);
	}

}
void Models::gaussNoise()
{

}