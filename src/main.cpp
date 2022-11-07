#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "Mapper.h"


int main(int argc, char* argv[])
{

	ros::init(argc, argv, "tsdf");

	TSDFMapper mapper;
	mapper.Init();

	ros::spin();

	return 0;
}
