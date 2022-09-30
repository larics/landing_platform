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
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>

#include <uav_ros_lib/nonlinear_filters.hpp>

#include <dynamic_reconfigure/server.h>
#include <landing_platform/LandingPlatformParametersConfig.h>


using namespace std;

class LandingPlatform
{
    public:
       	LandingPlatform();

        bool readyForStart();

        void doLanding();

        virtual void initializeParameters(ros::NodeHandle& nh);

        /**
         * Callback function used for setting various parameters.
         */
        void parametersCallback(
                landing_platform::LandingPlatformParametersConfig& configMsg,
                uint32_t level);
        /**
         * Set reconfigure parameters in the given config object.
         */
        void setReconfigureParameters(landing_platform::LandingPlatformParametersConfig& config);
        
        boost::recursive_mutex config_mutex;
        // Initialize configure server
        dynamic_reconfigure::Server<landing_platform::
            LandingPlatformParametersConfig>confServer{config_mutex};
        
        // Initialize reconfigure callback
        dynamic_reconfigure::Server<landing_platform::
            LandingPlatformParametersConfig>::CallbackType paramCallback;

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
         * Current Carrot reference data.
         */
        void referenceCallback(const geometry_msgs::PoseStamped::ConstPtr& referenceMsg);

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

        ros::Subscriber _odometrySub, _referenceSub, _outerTagSub, _innerTagSub;
        ros::Publisher _trackerPosePub, _detectedTagsPub;

        int _outerTagId;
        int _innerTagId;

        double _outerTagAltMax;
        double _outerTagAltMin;
        double _innerTagAltMax;
        double _innerTagAltMin;

        uint32_t _outerTagDetectionCounter;
        uint32_t _innerTagDetectionCounter;

        int _outerTagValidDetections;
        int _innerTagValidDetections;
        
        geometry_msgs::Point _outerTagPositionLocal, _innerTagPositionLocal;
        geometry_msgs::Quaternion _outerTagOrientationLocal, _innerTagOrientationLocal;
        double _outerTagOrientationLocalYaw, _innerTagOrientationLocalYaw;
        geometry_msgs::PointStamped _outerTagPositionGlobal, _outerTagPositionGlobalRaw, _innerTagPositionGlobal, _innerTagPositionGlobalRaw;
                                    
        nonlinear_filters::MedianFilter<double, 11> _medianOuterTagPosXGlobal, _medianOuterTagPosYGlobal, _medianOuterTagPosZGlobal, _medianOuterTagPosXLocal, _medianOuterTagPosYLocal, _medianOuterTagPosZLocal;
        nonlinear_filters::MedianFilter<double, 11> _medianOuterTagqX, _medianOuterTagqY, _medianOuterTagqZ, _medianOuterTagqW;
        nonlinear_filters::MedianFilter<double, 11> _medianInnerTagPosXGlobal, _medianInnerTagPosYGlobal, _medianInnerTagPosZGlobal, _medianInnerTagPosXLocal, _medianInnerTagPosYLocal, _medianInnerTagPosZLocal;
        nonlinear_filters::MedianFilter<double, 11> _medianInnerTagqX, _medianInnerTagqY, _medianInnerTagqZ, _medianInnerTagqW;
        
        geometry_msgs::Point _currentOdomPosition;

        geometry_msgs::Point _currentRefPositionFlight, _currentRefPosition;
        double _currentOdomImuYawFlight, _currentRefYaw;
        geometry_msgs::Quaternion _currentRefOrientation;
        bool _firstRefMsgFlag;

        double _qx, _qy, _qz, _qw;
        double _currentOdomImuYaw;

        tf::TransformListener listener;

        double _altitudeStep;
        double _maxHorizontalError;
        double _Kz;
        
        uint32_t _mpcPoseSentFlag;

        /** Define all the services */
		ros::ServiceServer _serviceLanding;

};

#endif