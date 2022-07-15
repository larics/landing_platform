#include "landing_platform/landing_platform.h"

/*
 * Default topics for remapping:
 * 		- /velodyne_points 	- INPUT  - PointCloud2 ROS message
 * 		- /uav/ref_pose	  	- OUTPUT - PoseStamped message representing the next 
 *          calculated waypoint
 */
int main(int argc, char **argv) 
{

	// Setup the node
	ros::init(argc, argv, "landing_platform");
	ros::NodeHandle nh;
	
	// Setup loop rate
	double rate = 10;
	bool initialized =
		nh.getParam("landing_platform_node/rate", rate);
	// Check if PotentialFields is properly initialized
	if (!initialized)
	{
		ROS_FATAL("LandingPlatformNode:: rate initialization failed.");
		throw std::invalid_argument("LandingPlatformNode parameters not properly set.");
	}
	ROS_INFO("LandingPlatformNode: Setting rate to %.2f", rate);
	ros::Rate loopRate {rate};

	// Setup a new potentialFields object
	LandingPlatform landing_platform;
	
	// Start the main loop
	while(ros::ok())
	{
		ros::spinOnce();
		
		if (landing_platform.readyForStart())
		{
			landing_platform.doLanding();
		}
		loopRate.sleep();

	}

	return 0;
}