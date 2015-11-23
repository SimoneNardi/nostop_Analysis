#include "Analysis.h"

#include "ros/ros.h"

#include <set>
#include <stdio.h>
#include <stdlib.h>

#include <memory>

////////////////////////////////////////////////////////////////
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Analysis");
	
	ROS_INFO("Analysis is running.");
	
	std::shared_ptr<Analysis> l_analysis = std::make_shared<Analysis>();
	l_analysis->Initialize();

	ros::spin();
	
	return 0;
}