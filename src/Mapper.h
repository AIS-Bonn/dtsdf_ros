//
// Created by Malte Splietker on 29.10.21.
//

#pragma once

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <ros/node_handle.h>
#include <sensor_msgs/Image.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <Eigen/Dense>

#include <ITMLib/Core/ITMMainEngine.h>

class TSDFMapper
{
public:
	TSDFMapper() : nh("~"), imageSync(1), cameraInfoSync(1), imageTransport(nh), tfListener(tfBuffer)
	{}

	void Init();

private:
	/**
	 * Initializes the TSDF mapping engine, once the camera parameters are known
	 * @return
	 */
	bool InitEngine(const sensor_msgs::CameraInfoConstPtr& depthCameraInfo,
	                const sensor_msgs::CameraInfoConstPtr& colorCameraInfo);

	void ImagePairCallback(const sensor_msgs::ImageConstPtr& depthMsg, const sensor_msgs::ImageConstPtr& colorMsg);

	void CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& depthMsg,
	                        const sensor_msgs::CameraInfoConstPtr& colorMsg);

	/** Publish rendered color image from current tracking pose  */
	void publishImage();

	/** Publish point cloud from current tracking pose */
	void publishPointCloud();

	/** Publish tracking result to tf tree */
	void publishTrackingResult();

	ITMLib::ITMMainEngine* mainEngine = nullptr;

	ros::NodeHandle nh;
	image_transport::ImageTransport imageTransport;
	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener;
	tf2_ros::TransformBroadcaster tfBroadcaster;

	// publishers
	image_transport::Publisher renderPublisher;
	ros::Publisher pointCloudPublisher;

	// subscribers
	image_transport::SubscriberFilter depthImageSubscriber;
	image_transport::SubscriberFilter colorImageSubscriber;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> ImageSyncPolicy;
	message_filters::Synchronizer<ImageSyncPolicy> imageSync;

	message_filters::Subscriber<sensor_msgs::CameraInfo> depthCameraInfoSubscriber;
	message_filters::Subscriber<sensor_msgs::CameraInfo> colorCameraInfoSubscriber;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> CameraInfoSyncPolicy;
	message_filters::Synchronizer<CameraInfoSyncPolicy> cameraInfoSync;

	/** Previous odometry pose, used to compute delta */
	Eigen::Affine3d previousOdometryPose;

	std_msgs::Header currentDepthHeader;

	/** Container for rendering and publishing point cloud */
	ITMLib::ITMPointCloud* pointCloud = nullptr;

	// Parameters
	/** Odometry frame (default is odom) */
	std::string odomFrameId;

	/** Map frame (default is map) */
	std::string globalFrameId;

	/** Robot base link frame (default is base_link) */
	std::string baseLinkFrameId;

	/** Child frame for publishing tf (default is odom).
	 * Required, if a different root frame above odom already exists.
	 **/
	std::string tfChildFrameId;

	/** Factor the depth values are scaled with (default is 1000) */
	float depthScaleFactor;

	/** Whether to publish tracking result as tf */
	bool broadcastTf;

	/** Use odometry to initialize pose estimate */
	bool useOdometry;

	/** Whether published point cloud also contains color */
	bool pointCloudWithColor;

};