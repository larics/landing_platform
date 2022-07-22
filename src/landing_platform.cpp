#include <ros/ros.h>
#include <ros/package.h>

#include <landing_platform/landing_platform.h>

LandingPlatform::LandingPlatform()
{
    //ROS Subscribers
	_odometrySub = nh.subscribe("odometry", 1, 
		&LandingPlatform::odometryCallback, this);

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
    initializeParameters(nh);
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

void LandingPlatform::outerTagDetectorCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr& outerTagMsg)
{
    uint32_t tagId;
    double tagPosSum, tagOriSum;
    geometry_msgs::Point tagPosition;
    geometry_msgs::Quaternion tagOrientation;
    std::vector<ar_track_alvar_msgs::AlvarMarker> tagList;
    // ROS_INFO("outer_tag");
    if ((_currentOdomPosition.z <= _outerTagAltMax) && (_currentOdomPosition.z >= _outerTagAltMin))
    {
        tagList = outerTagMsg->markers;
        for (auto& tag : tagList)
        {
            tagId = tag.id;
            // ROS_INFO("OuterTagID: %d", tagId);
            if (tagId == _outerTagId)
            {
                _outerTagDetectionCounter++;
                tagPosition = tag.pose.pose.position;
                tagOrientation = tag.pose.pose.orientation;
                _outerTagPosX.push_back(tagPosition.x);
                _outerTagPosY.push_back(tagPosition.y);
                _outerTagqX.push_back(tagOrientation.x);
                _outerTagqY.push_back(tagOrientation.y);
                _outerTagqZ.push_back(tagOrientation.z);
                _outerTagqW.push_back(tagOrientation.w);
                tagPosSum = std::accumulate(_outerTagPosX.begin(), _outerTagPosX.end(), 0.0);
                // ROS_INFO("SumX: %.2f", tagPosSum);
                _outerTagPositionX = tagPosSum/_outerTagDetectionCounter;
                tagPosSum = std::accumulate(_outerTagPosY.begin(), _outerTagPosY.end(), 0.0);
                // ROS_INFO("SumY: %.2f", tagPosSum);
                _outerTagPositionY = tagPosSum/_outerTagDetectionCounter;
                tagOriSum = std::accumulate(_outerTagqX.begin(), _outerTagqX.end(), 0.0);
                _outerTagOrientationX = tagOriSum/_outerTagDetectionCounter;
                tagOriSum = std::accumulate(_outerTagqY.begin(), _outerTagqY.end(), 0.0);
                _outerTagOrientationY = tagOriSum/_outerTagDetectionCounter;
                tagOriSum = std::accumulate(_outerTagqZ.begin(), _outerTagqZ.end(), 0.0);
                _outerTagOrientationZ = tagOriSum/_outerTagDetectionCounter;
                tagOriSum = std::accumulate(_outerTagqW.begin(), _outerTagqW.end(), 0.0);
                _outerTagOrientationW = tagOriSum/_outerTagDetectionCounter;
            }
            // ROS_INFO("_outerTagDetectionCounter: %d", _outerTagDetectionCounter);
            // ROS_INFO("_outerTagPosition: X:  %.2f, Y:  %.2f", _outerTagPositionX, _outerTagPositionY);           
        }
    }

}
void LandingPlatform::innerTagDetectorCallback(const ar_track_alvar_msgs::AlvarMarkersConstPtr& innerTagMsg)
{
    uint32_t tagId;
    double tagPosSum, tagOriSum;
    geometry_msgs::Point tagPosition;
    geometry_msgs::Quaternion tagOrientation;
    std::vector<ar_track_alvar_msgs::AlvarMarker> tagList;
    if ((_currentOdomPosition.z <= _innerTagAltMax) && (_currentOdomPosition.z >= _innerTagAltMin))
    {
        tagList = innerTagMsg->markers;
        for (auto& tag : tagList)
        {
            tagId = tag.id;
            // ROS_INFO("InnerTagID: %d", tagId);
            if (tagId == _innerTagId)
            {
                _innerTagDetectionCounter++;
                tagPosition = tag.pose.pose.position;
                tagOrientation = tag.pose.pose.orientation;
                _innerTagPosX.push_back(tagPosition.x);
                _innerTagPosY.push_back(tagPosition.y);
                _innerTagqX.push_back(tagOrientation.x);
                _innerTagqY.push_back(tagOrientation.y);
                _innerTagqZ.push_back(tagOrientation.z);
                _innerTagqW.push_back(tagOrientation.w);
                tagPosSum = std::accumulate(_innerTagPosX.begin(), _innerTagPosX.end(), 0.0);
                // ROS_INFO("SumX: %.2f", tagPosSum);
                _innerTagPositionX = tagPosSum/_innerTagDetectionCounter;
                tagPosSum = std::accumulate(_innerTagPosY.begin(), _innerTagPosY.end(), 0.0);
                // ROS_INFO("SumY: %.2f", tagPosSum);
                _innerTagPositionY = tagPosSum/_innerTagDetectionCounter;
                tagOriSum = std::accumulate(_innerTagqX.begin(), _innerTagqX.end(), 0.0);
                _innerTagOrientationX = tagOriSum/_innerTagDetectionCounter;
                tagOriSum = std::accumulate(_innerTagqY.begin(), _innerTagqY.end(), 0.0);
                _innerTagOrientationY = tagOriSum/_innerTagDetectionCounter;
                tagOriSum = std::accumulate(_innerTagqZ.begin(), _innerTagqZ.end(), 0.0);
                _innerTagOrientationZ = tagOriSum/_innerTagDetectionCounter;
                tagOriSum = std::accumulate(_innerTagqW.begin(), _innerTagqW.end(), 0.0);
                _innerTagOrientationW = tagOriSum/_innerTagDetectionCounter;
            }
            // ROS_INFO("_innerTagDetectionCounter: %d", _innerTagDetectionCounter);
            // ROS_INFO("_innerTagPosition: X:  %.2f, Y:  %.2f", _innerTagPositionX, _innerTagPositionY);
        }
    }

}

void LandingPlatform::doLanding()
{
    if (_landingPlatformToggled)
    {
        ROS_INFO("_outerTagPosition: X:  %.2f, Y:  %.2f", _outerTagPositionX, _outerTagPositionY);           
        ROS_INFO("_innerTagPosition: X:  %.2f, Y:  %.2f", _innerTagPositionX, _innerTagPositionY);
        if ((_innerTagDetectionCounter > _innerTagValidDetections) 
            && (_currentOdomPosition.z >= _innerTagAltMin)
            && (_currentOdomPosition.z <= _innerTagAltMax))
        {
            sendMpcTrackerPose(_innerTagId);
        }
        else if (_outerTagDetectionCounter > _outerTagValidDetections 
            && (_currentOdomPosition.z >= _outerTagAltMin)
            && (_currentOdomPosition.z <= _outerTagAltMax))
        {
            sendMpcTrackerPose(_outerTagId);
        }
    }

    publishDetectedTags();

}

void LandingPlatform::initializeParameters(ros::NodeHandle& nh)
{
    _outerTagId = 63;
    _innerTagId = 0;

    _outerTagDetectionCounter = 0;
    _innerTagDetectionCounter = 0;

    _outerTagAltMax = 30.0;
    _outerTagAltMin = 2.0;
    _innerTagAltMax = 5.0;
    _innerTagAltMin = 1.0;

    _outerTagDetectionCounter = 0;
    _innerTagDetectionCounter = 0;

    _outerTagValidDetections = 120;
    _innerTagValidDetections = 120;

    bool initialized = 
        nh.getParam("landing_platform_node/rate", _rate) &&
		nh.getParam("landing_platform_node/global_frame_id", _globalFrameID) &&
        nh.getParam("landing_platform_node/robot_frame_id", _robotFrameID);

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
        geometry_msgs::PoseStamped pose;
		pose.header.frame_id = _globalFrameID;
		pose.header.stamp = ros::Time::now();
		pose.pose.position.x = _outerTagPositionX;
		pose.pose.position.y = _outerTagPositionY;
        // pose.pose.position.x = _outerTagPositionY;      //Gazebo hack
		// pose.pose.position.y = -_outerTagPositionX;     //Gazebo hack
		pose.pose.position.z = _outerTagAltMin;
		pose.pose.orientation.x = _outerTagOrientationX;
		pose.pose.orientation.y = _outerTagOrientationY;
		pose.pose.orientation.z = _outerTagOrientationZ;
		pose.pose.orientation.w = _outerTagOrientationW;
		_trackerPosePub.publish(pose);
    } 
    else if (tagId == _innerTagId)
    {
        geometry_msgs::PoseStamped pose;
		pose.header.frame_id = _globalFrameID;
		pose.header.stamp = ros::Time::now();
        pose.pose.position.x = _innerTagPositionX;
		pose.pose.position.y = _innerTagPositionY;
		// pose.pose.position.x = _innerTagPositionY;      //Gazebo hack
		// pose.pose.position.y = -_innerTagPositionX;     //Gazebo hack
		pose.pose.position.z = _innerTagAltMin;
		pose.pose.orientation.x = _innerTagOrientationX;
		pose.pose.orientation.y = _innerTagOrientationY;
		pose.pose.orientation.z = _innerTagOrientationZ;
		pose.pose.orientation.w = _innerTagOrientationW;
		_trackerPosePub.publish(pose);
    }
    ROS_INFO("Pose sent to MPC Tracker");
}

void LandingPlatform::publishDetectedTags()
{
    landing_platform::DetectedTags msgDetectedTags;
    msgDetectedTags.outer_tag_id = _outerTagId;
    msgDetectedTags.outer_tag_valid_detections = _outerTagDetectionCounter;
    msgDetectedTags.outer_tag_position.x = _outerTagPositionX;
    msgDetectedTags.outer_tag_position.y = _outerTagPositionY;
    msgDetectedTags.outer_tag_position.z = _outerTagPositionZ;
    msgDetectedTags.outer_tag_orinentation.x = _outerTagOrientationX;
    msgDetectedTags.outer_tag_orinentation.y = _outerTagOrientationY;
    msgDetectedTags.outer_tag_orinentation.z = _outerTagOrientationZ;
    msgDetectedTags.outer_tag_orinentation.w = _outerTagOrientationW;
    msgDetectedTags.inner_tag_id = _innerTagId;
    msgDetectedTags.inner_tag_valid_detections = _innerTagDetectionCounter;
    msgDetectedTags.inner_tag_position.x = _innerTagPositionX;
    msgDetectedTags.inner_tag_position.y = _innerTagPositionY;
    msgDetectedTags.inner_tag_position.z = _innerTagPositionZ;
    msgDetectedTags.inner_tag_orinentation.x = _innerTagOrientationX;
    msgDetectedTags.inner_tag_orinentation.y = _innerTagOrientationY;
    msgDetectedTags.inner_tag_orinentation.z = _innerTagOrientationZ;
    msgDetectedTags.inner_tag_orinentation.w = _innerTagOrientationW;
    _detectedTagsPub.publish(msgDetectedTags);
}