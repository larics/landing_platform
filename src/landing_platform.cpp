#include <ros/ros.h>
#include <ros/package.h>

#include <landing_platform/landing_platform.h>

#include <uav_ros_lib/ros_convert.hpp>

LandingPlatform::LandingPlatform()
{
    //ROS Subscribers
	_odometrySub = nh.subscribe("odometry", 1, 
		&LandingPlatform::odometryCallback, this);
    _referenceSub = nh.subscribe("carrot/pose", 1, 
		&LandingPlatform::referenceCallback, this);

    _outerTagSub = nh.subscribe("/outer_tag/ar_pose_marker", 1,
     &LandingPlatform::outerTagDetectorCallback, this);
    _innerTagSub = nh.subscribe("/inner_tag/ar_pose_marker", 1,
     &LandingPlatform::innerTagDetectorCallback, this);

    //ROS Publishers
    _trackerPosePub = nh.advertise<geometry_msgs::PoseStamped>("mpc_tracker/pose", 1);
    _detectedTagsPub = nh.advertise<landing_platform::DetectedTags>("detected_tags", 1); 

    //ROS Services
	_serviceLanding = nh.advertiseService("landing_platform/toggle",
		&LandingPlatform::toggleLandingServiceCb, this);

    _startFlag = false;
    _landingPlatformToggled = false;
    _firstRefMsgFlag = true;
    initializeParameters(nh);
    landing_platform::LandingPlatformParametersConfig config;
	setReconfigureParameters(config);
	confServer.updateConfig(config);
	paramCallback = boost::bind(
		&LandingPlatform::parametersCallback, this, _1, _2);
	confServer.setCallback(paramCallback);
}

bool LandingPlatform::readyForStart()
{
	return _startFlag;
}

void LandingPlatform::odometryCallback(const nav_msgs::Odometry::ConstPtr& odometryMsg)
{
	_currentOdomPosition = odometryMsg->pose.pose.position;
	// Orientation data retrieval in quaternion
	_qx = odometryMsg->pose.pose.orientation.x;
	_qy = odometryMsg->pose.pose.orientation.y;
	_qz = odometryMsg->pose.pose.orientation.z;
	_qw = odometryMsg->pose.pose.orientation.w;
	_currentOdomImuYaw = atan2(2 * (_qw * _qz + _qx * _qy), _qw * _qw + _qx * _qx - _qy * _qy - _qz * _qz);	 	
    _startFlag = true;

}

void LandingPlatform::referenceCallback(const geometry_msgs::PoseStamped::ConstPtr& referenceMsg)
{
    //  ROS_INFO("_currentRefPositionFlight: X:  %.2f, Y:  %.2f, Z:  %.2f", _currentRefPositionFlight.x, _currentRefPositionFlight.y, _currentRefPositionFlight.z); 

	_currentRefPositionFlight = referenceMsg->pose.position;
	// Orientation data retrieval in quaternion
	geometry_msgs::Quaternion currentRefOrientationFlight = referenceMsg->pose.orientation;
	_currentOdomImuYawFlight = ros_convert::calculateYaw(currentRefOrientationFlight);

}

void LandingPlatform::outerTagDetectorCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr& outerTagMsg)
{
    uint32_t tagId;
    double tagPosSum, tagOriSum;
    geometry_msgs::Point tagPosition;
    geometry_msgs::PointStamped tagPositionStamped;
    geometry_msgs::Quaternion tagOrientation;
    std::vector<ar_track_alvar_msgs::AlvarMarker> tagList;
    
    // Transform detected tag position to base_link frame
	try
	{
    ros::Time now = ros::Time::now();
	tf::StampedTransform transform;
    listener.waitForTransform(_globalFrameID, _robotFrameID, now, ros::Duration(2.0));
    listener.lookupTransform(_globalFrameID, _robotFrameID, now, transform);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
    }

    if ((_outerTagPositionLocal.z <= _outerTagAltMax))
    {
        tagList = outerTagMsg->markers;
        for (auto& tag : tagList)
        {
            tagId = tag.id;
            if (tagId == _outerTagId)
            {
                _outerTagDetectionCounter++;
                tagPosition = tag.pose.pose.position;
                tagOrientation = tag.pose.pose.orientation;

                tagPositionStamped.header.frame_id = _robotFrameID;
                tagPositionStamped.point.x = tagPosition.x;
                tagPositionStamped.point.y = tagPosition.y;
                tagPositionStamped.point.z = tagPosition.z;
                listener.transformPoint(_globalFrameID, tagPositionStamped, _outerTagPositionGlobalRaw);

                _medianOuterTagPosXGlobal.addSample(_outerTagPositionGlobalRaw.point.x);
                _outerTagPositionGlobal.point.x = _medianOuterTagPosXGlobal.getMedian();
                _medianOuterTagPosYGlobal.addSample(_outerTagPositionGlobalRaw.point.y);
                _outerTagPositionGlobal.point.y = _medianOuterTagPosYGlobal.getMedian();
                _medianOuterTagPosZGlobal.addSample(_outerTagPositionGlobalRaw.point.z);
                _outerTagPositionGlobal.point.z = _medianOuterTagPosZGlobal.getMedian();
                _medianOuterTagPosXLocal.addSample(tagPosition.x);
                _outerTagPositionLocal.x = _medianOuterTagPosXLocal.getMedian();
                _medianOuterTagPosYLocal.addSample(tagPosition.y);
                _outerTagPositionLocal.y = _medianOuterTagPosYLocal.getMedian();
                _medianOuterTagPosZLocal.addSample(tagPosition.z);
                _outerTagPositionLocal.z = _medianOuterTagPosZLocal.getMedian();
                _medianOuterTagqX.addSample(tagOrientation.x);
                _outerTagOrientationLocal.x = _medianOuterTagqX.getMedian();
                _medianOuterTagqY.addSample(tagOrientation.y);
                _outerTagOrientationLocal.y = _medianOuterTagqY.getMedian();
                _medianOuterTagqZ.addSample(tagOrientation.z);
                _outerTagOrientationLocal.z = _medianOuterTagqZ.getMedian();
                _medianOuterTagqW.addSample(tagOrientation.w);
                _outerTagOrientationLocal.w = _medianOuterTagqW.getMedian();
                _outerTagOrientationLocalYaw = ros_convert::calculateYaw(_outerTagOrientationLocal.x, _outerTagOrientationLocal.y, _outerTagOrientationLocal.z, _outerTagOrientationLocal.w);
            }
        }
    }
}

void LandingPlatform::innerTagDetectorCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr& innerTagMsg)
{
    uint32_t tagId;
    double tagPosSum, tagOriSum;
    geometry_msgs::Point tagPosition;
    geometry_msgs::PointStamped tagPositionStamped;
    geometry_msgs::Quaternion tagOrientation;
    std::vector<ar_track_alvar_msgs::AlvarMarker> tagList;

    // Transform detected tag position to base_link frame
	try
	{
    ros::Time now = ros::Time::now();
	tf::StampedTransform transform;
    listener.waitForTransform(_globalFrameID, _robotFrameID,  now, ros::Duration(2.0));
    listener.lookupTransform(_globalFrameID, _robotFrameID, now, transform);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
    }
    if ((_innerTagPositionLocal.z <= _innerTagAltMax))// && (_innerTagPositionLocal.point.z >= _innerTagAltMin))
    {
        tagList = innerTagMsg->markers;
        for (auto& tag : tagList)
        {
            tagId = tag.id;
            if (tagId == _innerTagId)
            {
                _innerTagDetectionCounter++;
                tagPosition = tag.pose.pose.position;
                tagOrientation = tag.pose.pose.orientation;

                tagPositionStamped.header.frame_id = _robotFrameID;
                tagPositionStamped.point.x = tagPosition.x;
                tagPositionStamped.point.y = tagPosition.y;
                tagPositionStamped.point.z = tagPosition.z;
                listener.transformPoint(_globalFrameID, tagPositionStamped, _innerTagPositionGlobalRaw);

            
                _medianInnerTagPosXGlobal.addSample(_innerTagPositionGlobalRaw.point.x);
                _innerTagPositionGlobal.point.x = _medianInnerTagPosXGlobal.getMedian();
                _medianInnerTagPosYGlobal.addSample(_innerTagPositionGlobalRaw.point.y);
                _innerTagPositionGlobal.point.y = _medianInnerTagPosYGlobal.getMedian();
                _medianInnerTagPosZGlobal.addSample(_innerTagPositionGlobalRaw.point.z);
                _innerTagPositionGlobal.point.z = _medianInnerTagPosZGlobal.getMedian();
                _medianInnerTagPosXLocal.addSample(tagPosition.x);
                _innerTagPositionLocal.x = _medianInnerTagPosXLocal.getMedian();
                _medianInnerTagPosYLocal.addSample(tagPosition.y);
                _innerTagPositionLocal.y = _medianInnerTagPosYLocal.getMedian();
                _medianInnerTagPosZLocal.addSample(tagPosition.z);
                _innerTagPositionLocal.z = _medianInnerTagPosZLocal.getMedian();
                _medianInnerTagqX.addSample(tagOrientation.x);
                _innerTagOrientationLocal.x = _medianInnerTagqX.getMedian();
                _medianInnerTagqY.addSample(tagOrientation.y);
                _innerTagOrientationLocal.y = _medianInnerTagqY.getMedian();
                _medianInnerTagqZ.addSample(tagOrientation.z);
                _innerTagOrientationLocal.z = _medianInnerTagqZ.getMedian();
                _medianInnerTagqW.addSample(tagOrientation.w);
                _innerTagOrientationLocal.w = _medianInnerTagqW.getMedian();
                _innerTagOrientationLocalYaw = ros_convert::calculateYaw(_innerTagOrientationLocal.x, _innerTagOrientationLocal.y, _innerTagOrientationLocal.z, _innerTagOrientationLocal.w);
          }
       }
    }

}

void LandingPlatform::doLanding()
{
    if (_landingPlatformToggled)
    {
        ROS_INFO("OuterTag valid detections:  %.d", _outerTagDetectionCounter);           
        ROS_INFO("_outerTagPosition: X:  %.2f, Y:  %.2f, Z:  %.2f", _outerTagPositionGlobal.point.x, _outerTagPositionGlobal.point.y, _outerTagPositionGlobal.point.z); 
        ROS_INFO("_outerTagPosition: XLocal:  %.2f, YLocal:  %.2f, ZLocal:  %.2f", _outerTagPositionLocal.x, _outerTagPositionLocal.y, _outerTagPositionLocal.z);           
        ROS_INFO("InnterTag valid detections:  %.d", _innerTagDetectionCounter);  
        ROS_INFO("_innerTagPosition: X:  %.2f, Y:  %.2f, Z:  %.2f", _innerTagPositionGlobal.point.x, _innerTagPositionGlobal.point.y, _innerTagPositionGlobal.point.z);
        ROS_INFO("_innerTagPosition: XLocal:  %.2f, YLocal:  %.2f, ZLocal:  %.2f", _innerTagPositionLocal.x, _innerTagPositionLocal.y, _innerTagPositionLocal.z);           

        // if ((_innerTagDetectionCounter > _innerTagValidDetections) 
        //     && (abs(_innerTagPositionLocal.z) > _innerTagAltMin)
        //     && (abs(_innerTagPositionLocal.z) < _innerTagAltMax))
        // {
        //     sendMpcTrackerPose(_innerTagId);
        // }
        // else if (_outerTagDetectionCounter > _outerTagValidDetections 
        //     && (abs(_outerTagPositionLocal.z) > _outerTagAltMin)
        //     && (abs(_outerTagPositionLocal.z) < _outerTagAltMax))
        // {
        //     sendMpcTrackerPose(_outerTagId);
        // }
        if ((_innerTagDetectionCounter > _innerTagValidDetections) 
            && (abs(_innerTagPositionLocal.z) < _innerTagAltMax))
        {
            sendMpcTrackerPose(_innerTagId);
        }
        else if (_outerTagDetectionCounter > _outerTagValidDetections 
            && (abs(_outerTagPositionLocal.z) < _outerTagAltMax))
        {
            sendMpcTrackerPose(_outerTagId);
        }
    }

    publishDetectedTags();

}

void LandingPlatform::parametersCallback(
	landing_platform::LandingPlatformParametersConfig& configMsg,
	uint32_t level)
{
	ROS_WARN("Hello from LandingPlatform parameters callback.");
	_outerTagId = configMsg.outer_tag_id;
    _innerTagId = configMsg.inner_tag_id;
    _outerTagAltMax = configMsg.outer_tag_max_alt;
    _outerTagAltMin = configMsg.outer_tag_min_alt;
    _innerTagAltMax = configMsg.inner_tag_max_alt;
    _innerTagAltMin = configMsg.inner_tag_min_alt;
    _outerTagValidDetections = configMsg.outer_tag_valid_detection;
    _innerTagValidDetections = configMsg.inner_tag_valid_detection;
    _altitudeStep = configMsg.altitude_step;
    _Kz = configMsg.k_z;
}

void LandingPlatform::setReconfigureParameters(landing_platform::LandingPlatformParametersConfig& config)
{
	ROS_WARN("LandingPlatform - Reconfigure parameters called.");
	config.outer_tag_id = _outerTagId;
    config.inner_tag_id = _innerTagId;
    config.outer_tag_max_alt = _outerTagAltMax;
    config.outer_tag_min_alt = _outerTagAltMin;
    config.inner_tag_max_alt = _innerTagAltMax;
    config.inner_tag_min_alt = _innerTagAltMin;
    config.outer_tag_valid_detection = _outerTagValidDetections;
    config.inner_tag_valid_detection = _innerTagValidDetections;
    config.altitude_step = _altitudeStep;
    config.k_z = _Kz;
}

void LandingPlatform::initializeParameters(ros::NodeHandle& nh)
{
    // _outerTagId = 63;
    // _innerTagId = 0;

    // _outerTagAltMax = 12.0;
    // _outerTagAltMin = 2.0;
    // _innerTagAltMax = 4.0;
    // _innerTagAltMin = 1.0;

    // _outerTagValidDetections = 120;
    // _innerTagValidDetections = 120;

    // _altitudeStep = 0.01;
    // // // _maxHorizontalError = 0.1;
    // // // _Kz = 0.01;

    bool initialized = 
        nh.getParam("landing_platform_node/rate", _rate) &&
		nh.getParam("landing_platform_node/global_frame_id", _globalFrameID) &&
        nh.getParam("landing_platform_node/robot_frame_id", _robotFrameID) &&
        nh.getParam("landing_platform_node/outer_tag_id", _outerTagId) &&
        nh.getParam("landing_platform_node/inner_tag_id", _innerTagId) &&
        nh.getParam("landing_platform_node/outer_tag_max_alt", _outerTagAltMax) &&
        nh.getParam("landing_platform_node/outer_tag_min_alt", _outerTagAltMin) &&
        nh.getParam("landing_platform_node/inner_tag_max_alt", _innerTagAltMax) &&
        nh.getParam("landing_platform_node/inner_tag_min_alt", _innerTagAltMin) &&
        nh.getParam("landing_platform_node/outer_tag_valid_detection", _outerTagValidDetections) &&
        nh.getParam("landing_platform_node/inner_tag_valid_detection", _innerTagValidDetections) &&
        nh.getParam("landing_platform_node/altitude_step", _altitudeStep) &&
        nh.getParam("landing_platform_node/k_z", _Kz);

    _outerTagDetectionCounter = 0;
    _innerTagDetectionCounter = 0;
    _mpcPoseSentFlag = 0;

    // Check if LandingPlatform is properly initialized
	if (!initialized)
	{
		ROS_FATAL("LandingPlatform::initializeParameters() - parameter initialization failed.");
		throw std::invalid_argument("LandingPlatform parameters not properly set.");
	}
}

void LandingPlatform::sendMpcTrackerPose(int tagId)
{
    if (tagId == _outerTagId)
    {
        if (_firstRefMsgFlag)
        {
        _currentRefPosition = _currentRefPositionFlight;
        _currentRefYaw = _currentOdomImuYawFlight;
        _firstRefMsgFlag = false;
        }
        double refYaw = _currentOdomImuYaw + _outerTagOrientationLocalYaw;
        // double refYaw = _currentRefYaw + _outerTagOrientationLocalYaw;
        _currentRefYaw = ros_convert::wrapMinMax(refYaw, -M_PI, M_PI);
        _currentRefOrientation = ros_convert::calculate_quaternion(_currentRefYaw);
        _currentRefPosition.x = _outerTagPositionGlobal.point.x;
        _currentRefPosition.y = _outerTagPositionGlobal.point.y;
        _currentRefPosition.z = _currentRefPosition.z + _Kz*(_outerTagAltMin + _outerTagPositionLocal.z);
        // if (abs(_outerTagPositionLocal.z) > _outerTagAltMin)
        // {
        //     // _currentRefPosition.z = max(_currentRefPosition.z - _altitudeStep, _outerTagAltMin);
        //     _currentRefPosition.z = _currentRefPosition.z - _altitudeStep;
        // }
        // else
        // {
        //     _currentRefPosition.z = _outerTagAltMin;
        // }
        _mpcPoseSentFlag = 1;
    } 
    else if (tagId == _innerTagId)
    {
        double refYaw = _currentOdomImuYaw + _innerTagOrientationLocalYaw;
        // double refYaw = _currentRefYaw + _innerTagOrientationLocalYaw;
        _currentRefYaw = ros_convert::wrapMinMax(refYaw, -M_PI, M_PI);
        _currentRefOrientation = ros_convert::calculate_quaternion(_currentRefYaw);    
        _currentRefPosition.x = _innerTagPositionGlobal.point.x;
        _currentRefPosition.y = _innerTagPositionGlobal.point.y;
        _currentRefPosition.z = _currentRefPosition.z + _Kz*(_innerTagAltMin + _innerTagPositionLocal.z);
        // if (abs(_innerTagPositionLocal.z) > _innerTagAltMin)
        // {
        //     // _currentRefPosition.z = max(_currentRefPosition.z - _altitudeStep, _innerTagAltMin);
        //     _currentRefPosition.z = _currentRefPosition.z - _altitudeStep;
        // }
        // else
        // {
        //     _currentRefPosition.z = _innerTagAltMin;
        // }    
        _mpcPoseSentFlag = 2;
    }
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = _globalFrameID;
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = _currentRefPosition.x;
    pose.pose.position.y = _currentRefPosition.y;
    pose.pose.position.z = _currentRefPosition.z;
    pose.pose.orientation.x = _currentRefOrientation.x;
    pose.pose.orientation.y = _currentRefOrientation.y;
    pose.pose.orientation.z = _currentRefOrientation.z;
    pose.pose.orientation.w = _currentRefOrientation.w;
    _trackerPosePub.publish(pose);
    ROS_INFO("Pose sent to MPC Tracker");
}

void LandingPlatform::publishDetectedTags()
{
    landing_platform::DetectedTags msgDetectedTags;
    msgDetectedTags.outer_tag_id = _outerTagId;
    msgDetectedTags.outer_tag_valid_detections = _outerTagDetectionCounter;
    msgDetectedTags.outer_tag_position_global.x = _outerTagPositionGlobal.point.x;
    msgDetectedTags.outer_tag_position_global.y = _outerTagPositionGlobal.point.y;
    msgDetectedTags.outer_tag_position_global.z = _outerTagPositionGlobal.point.z;
    msgDetectedTags.outer_tag_position_local.x = _outerTagPositionLocal.x;
    msgDetectedTags.outer_tag_position_local.y = _outerTagPositionLocal.y;
    msgDetectedTags.outer_tag_position_local.z = _outerTagPositionLocal.z;
    msgDetectedTags.outer_tag_orinentation_local.x = _outerTagOrientationLocal.x;
    msgDetectedTags.outer_tag_orinentation_local.y = _outerTagOrientationLocal.y;
    msgDetectedTags.outer_tag_orinentation_local.z = _outerTagOrientationLocal.z;
    msgDetectedTags.outer_tag_orinentation_local.w = _outerTagOrientationLocal.w;
    msgDetectedTags.outer_tag_yaw = _outerTagOrientationLocalYaw*180/M_PI;
    msgDetectedTags.inner_tag_id = _innerTagId;
    msgDetectedTags.inner_tag_valid_detections = _innerTagDetectionCounter;
    msgDetectedTags.inner_tag_position_global.x = _innerTagPositionGlobal.point.x;
    msgDetectedTags.inner_tag_position_global.y = _innerTagPositionGlobal.point.y;
    msgDetectedTags.inner_tag_position_global.z = _innerTagPositionGlobal.point.z;
    msgDetectedTags.inner_tag_position_local.x = _innerTagPositionLocal.x;
    msgDetectedTags.inner_tag_position_local.y = _innerTagPositionLocal.y;
    msgDetectedTags.inner_tag_position_local.z = _innerTagPositionLocal.z;
    msgDetectedTags.inner_tag_orinentation_local.x = _innerTagOrientationLocal.x;
    msgDetectedTags.inner_tag_orinentation_local.y = _innerTagOrientationLocal.y;
    msgDetectedTags.inner_tag_orinentation_local.z = _innerTagOrientationLocal.z;
    msgDetectedTags.inner_tag_orinentation_local.w = _innerTagOrientationLocal.w;
    msgDetectedTags.inner_tag_yaw = _innerTagOrientationLocalYaw*180/M_PI;
    msgDetectedTags.reference_position.x = _currentRefPosition.x;
    msgDetectedTags.reference_position.y = _currentRefPosition.y;
    msgDetectedTags.reference_position.z = _currentRefPosition.z;
    msgDetectedTags.reference_yaw = _currentRefYaw*180/M_PI;
    msgDetectedTags.mpc_pose_sent_flag = _mpcPoseSentFlag;
    _detectedTagsPub.publish(msgDetectedTags);
    
    _mpcPoseSentFlag = 0;
}