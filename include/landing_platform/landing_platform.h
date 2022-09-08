#ifndef _LANDING_PLATFORM_H
#define _LANDING_PLATFORM_H

#include <string>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <vector>
#include <math.h>
#include <algorithm>
#include <numeric>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <ar_track_alvar_msgs/AlvarMarkers.h>

#include <landing_platform/DetectedTags.h>

#include <tf2/convert.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>

#include <uav_ros_lib/nonlinear_filters.hpp>


using namespace std;

class LandingPlatform
{
    public:
       	LandingPlatform();

        bool readyForStart();

        void doLanding();

        virtual void initializeParameters(ros::NodeHandle& nh);

        void sendMpcTrackerPose(int tagId);

        void publishDetectedTags();

		bool toggleLandingServiceCb(std_srvs::SetBool::Request& request, 
			std_srvs::SetBool::Response& response)
        {
            _landingPlatformToggled = request.data;
            if (_landingPlatformToggled)
            {
                std::cout << "Landing ON." << std::endl << std::endl;
                _startFlag = false;
            }
            else std::cout << "Landing OFF." << std::endl << std::endl;
            response.success = true;
	        response.message = "toggleLandingService called!";
            return true;
        }

        /**
         * Current Odometry data.
         */
        void odometryCallback(const nav_msgs::Odometry::ConstPtr& odometryMsg);

        /**
         * Alvar AR Tag detector subscribers
         */

        void outerTagDetectorCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr& outerTagMsg);
        void innerTagDetectorCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr& innerTagMsg);

    private:
        ros::NodeHandle nh, nhPrivate;
        int _rate;

        std::string _globalFrameID, _robotFrameID;

        bool _landingPlatformToggled;
        bool _startFlag;

        ros::Subscriber _odometrySub, _outerTagSub, _innerTagSub;
        ros::Publisher _trackerPosePub, _detectedTagsPub;

        int _outerTagId;
        int _innerTagId;

        double _outerTagAltMax;
        double _outerTagAltMin;
        double _innerTagAltMax;
        double _innerTagAltMin;

        uint32_t _outerTagDetectionCounter;
        uint32_t _innerTagDetectionCounter;

        uint32_t _outerTagValidDetections;
        uint32_t _innerTagValidDetections;
        
        std::vector<double> _outerTagPosX, _outerTagPosY, _outerTagPosZ, _innerTagPosX, _innerTagPosY, _innerTagPosZ;
        double _outerTagPositionX, _outerTagPositionY, _outerTagPositionZ, _innerTagPositionX, _innerTagPositionY, _innerTagPositionZ;
        std::vector<double> _outerTagqX, _outerTagqY, _outerTagqZ, _outerTagqW, _innerTagqX, _innerTagqY, _innerTagqZ, _innerTagqW;
        double _outerTagOrientationX, _outerTagOrientationY, _outerTagOrientationZ, _outerTagOrientationW, _innerTagOrientationX, _innerTagOrientationY, _innerTagOrientationZ, _innerTagOrientationW;

        nonlinear_filters::MedianFilter<double, 101> _medianOuterTagPosX, _medianOuterTagPosY, _medianOuterTagPosZ;
        nonlinear_filters::MedianFilter<double, 101> _medianOuterTagqX, _medianOuterTagqY, _medianOuterTagqZ, _medianOuterTagqW;
        nonlinear_filters::MedianFilter<double, 101> _medianInnerTagPosX, _medianInnerTagPosY, _medianInnerTagPosZ;
        nonlinear_filters::MedianFilter<double, 101> _medianInnerTagqX, _medianInnerTagqY, _medianInnerTagqZ, _medianInnerTagqW;

        
        geometry_msgs::Point _currentOdomPosition;
        double _qx, _qy, _qz, _qw;
        double _currentOdomImuYaw;
        

        /** Define all the services */
		ros::ServiceServer _serviceLanding;

};

#endif