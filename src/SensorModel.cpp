#include "bayes_filter/SensorModel.h"


// SensorModel::SensorModel(nav_msgs::OccupancyGrid map_):
// map(&map_)
// {

// 	std::cout << " sensor model map " << map->info.width << std::endl;
// }

SensorModel::SensorModel()
{}

// double SensorModel::getDistance(double x, double y, double xp, yp){
// 	return 
// }

void SensorModel::mapUpdate(nav_msgs::OccupancyGrid map_)
{
	map = map_;

}

double SensorModel::_findMinDistance(double x, double y){
	double dist = 100;
	double current_dist;
	for(auto obj:occupied_){
		// dist = getDistance(double x, double y, double obj.first, double obj.second);
		current_dist = sqrt(pow(x - obj.first, 2) + pow(y - obj.second, 2));

		if(dist>current_dist) dist = current_dist;
	}
	return dist;
}

double SensorModel::gaussianDistribution(double mean, double standardDeviation, double x){
	double probabilityX = (1 / (std::sqrt(2*_pi)*standardDeviation)) * std::exp(-1*(x-mean)*(x-mean) / (2*standardDeviation*standardDeviation));
	return probabilityX;
}

void SensorModel::measurementModel(std::vector<Particles> &samples, sensor_msgs::LaserScan scan, nav_msgs::OccupancyGrid map){
	
	double weight = 1.0;
	occupied_ = _find_objects(map);

	double zxlxt;
	double zylxt;

	double distance;
	for(auto &p:samples){
		int index = p.y * map.info.width + p.x;
		if((map.data[index] != 0) || map.data[index] != 1) continue;

		for(int i = 320; i<960; i++){
			double zlxt_angle = p.yaw - _pi + i * scan.angle_increment;
			zxlxt = p.x + scan.ranges[i] * sin(zlxt_angle);
			zylxt = p.y + scan.ranges[i] * cos(zlxt_angle);
			distance = _findMinDistance(zxlxt, zylxt);

			weight +=  _zhit * gaussianDistribution(0, 0.7, distance);
		}

		p.weight = weight;
	}


}


std::vector<std::pair<int,int>>  SensorModel::_find_objects(nav_msgs::OccupancyGrid map){
	
	std::vector<std::pair<int,int>> oc;
	for (int i = 0; i < map.info.width; i++){
		for (int j = 0; j < map.info.height; j++){
			size_t index = j * map.info.width + i;
			// if (index >= sizeof(map.data) || (index < -sizeof(map.data)))
			// 	continue;
			if ((int)map.data[index] == 100)
				oc.push_back(std::make_pair(i, j));
		}
	}
	return oc;
} 