//
// Created by Malte Splietker on 29.10.21.
//

#include "Mapper.h"

#include <ITMLib/Core/ITMBasicEngine.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_eigen/tf2_eigen.h>
#include <ORUtils/EigenConversion.h>
#include <ORUtils/SE3Pose.h>


#include <opencv2/opencv.hpp>

void TSDFMapper::Init()
{
	depthImageSubscriber.subscribe(imageTransport, ros::names::remap("/depth"), 10,
	                               image_transport::TransportHints("raw",
	                                                               ros::TransportHints(),
	                                                               nh,
	                                                               "image_transport_depth"));
	colorImageSubscriber.subscribe(imageTransport, ros::names::remap("/color"), 10,
	                               image_transport::TransportHints("raw",
	                                                               ros::TransportHints(),
	                                                               nh,
	                                                               "image_transport_color"));
	imageSync.connectInput(depthImageSubscriber, colorImageSubscriber);
	imageSync.registerCallback(&TSDFMapper::ImagePairCallback, this);

	depthCameraInfoSubscriber.subscribe(nh, "/camera_info_depth", 1);
	colorCameraInfoSubscriber.subscribe(nh, "/camera_info_color", 1);
	cameraInfoSync.connectInput(depthCameraInfoSubscriber, colorCameraInfoSubscriber);
	cameraInfoSync.registerCallback(&TSDFMapper::CameraInfoCallback, this);

	renderPublisher = imageTransport.advertise("rendered_view", 1);
	pointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);

	nh.param("odom_frame_id", odomFrameId, std::string("odom"));
	nh.param("global_frame_id", globalFrameId, std::string("map"));
	nh.param("base_link_frame_id", baseLinkFrameId, std::string("base_link"));
	nh.param("tf_child_frame_id", tfChildFrameId, std::string("odom"));
	nh.param("depth_scale_factor", depthScaleFactor, 1000.0f);
	nh.param("broadcast_tf", broadcastTf, true);
	nh.param("use_odometry", useOdometry, true);
	nh.param("point_cloud_with_color", pointCloudWithColor, false);
}

bool TSDFMapper::InitEngine(const sensor_msgs::CameraInfoConstPtr& depthCameraInfo,
                            const sensor_msgs::CameraInfoConstPtr& colorCameraInfo)
{
	if (depthCameraInfo->header.frame_id.empty() or colorCameraInfo->header.frame_id.empty())
		return false;

	ITMLib::ITMRGBDCalib calib;

	calib.intrinsics_d.SetFrom(depthCameraInfo->width, depthCameraInfo->height, depthCameraInfo->K[0],
	                           depthCameraInfo->K[4], depthCameraInfo->K[2], depthCameraInfo->K[5]);
	calib.disparityCalib.SetFrom(1.0f / depthScaleFactor, 0.0f, ITMLib::ITMDisparityCalib::TRAFO_AFFINE);
	calib.intrinsics_rgb.SetFrom(colorCameraInfo->width, colorCameraInfo->height, colorCameraInfo->K[0],
	                             colorCameraInfo->K[4], colorCameraInfo->K[2], colorCameraInfo->K[5]);

	Eigen::Affine3d initialPose;
	try
	{
		geometry_msgs::TransformStamped transform = tfBuffer.lookupTransform(
			colorCameraInfo->header.frame_id, depthCameraInfo->header.frame_id, ros::Time(0));
		Eigen::Affine3f extrinsics = tf2::transformToEigen(transform).cast<float>();
		ITMLib::Matrix4f extrinsicsMat = ORUtils::FromEigen<ITMLib::Matrix4f>(extrinsics.matrix());
		calib.trafo_rgb_to_depth.SetFrom(extrinsicsMat);

		transform = tfBuffer.lookupTransform(baseLinkFrameId, depthCameraInfo->header.frame_id, ros::Time(0));
		initialPose = tf2::transformToEigen(transform);

		transform = tfBuffer.lookupTransform(depthCameraInfo->header.frame_id, odomFrameId, ros::Time(0));
		previousOdometryPose = tf2::transformToEigen(transform);
	}
	catch (tf2::TransformException& ex)
	{
		ROS_WARN("%s", ex.what());
		return false;
	}

	std::string settingsFile;
	nh.getParam("settings_file", settingsFile);
	std::shared_ptr<ITMLib::ITMLibSettings> internalSettings;
	if (settingsFile.empty())
		internalSettings = std::make_shared<ITMLib::ITMLibSettings>();
	else
		internalSettings = std::make_shared<ITMLib::ITMLibSettings>(settingsFile);

	if (useOdometry)
		internalSettings->refinePoses = true;  // use ICP to refine poses from odometry

	ROS_INFO("Initializing MainEngine");
	auto* engine = new ITMLib::ITMBasicEngine(internalSettings, calib,
	                                          calib.intrinsics_d.imgSize,
	                                          calib.intrinsics_rgb.imgSize);

	// Use baselink-camera transform to initialize pose -> map frame starts at current base_link position
	engine->GetTrackingState()->pose_d->SetInvM(ORUtils::FromEigen<ITMLib::Matrix4f>(initialPose.matrix()));

	mainEngine = engine; // assignment here, so mainEngine can be used to determine if initialization complete.

	return true;
}

void ITMToCV(const ITMLib::ITMUChar4Image& src, cv::Mat& dst)
{
	dst = cv::Mat(src.noDims.height, src.noDims.width, CV_8UC4);

	const ITMLib::Vector4u* inPtr = src.GetData(MEMORYDEVICE_CPU);
	for (int r = 0; r < src.noDims.height; ++r)
	{
		for (int c = 0; c < src.noDims.width; ++c)
		{
			dst.at<cv::Vec4b>(r, c) = static_cast<cv::Vec4b>(inPtr[r * src.noDims.width + c]);
		}
	}
}

void
TSDFMapper::ImagePairCallback(const sensor_msgs::ImageConstPtr& depthMsg, const sensor_msgs::ImageConstPtr& colorMsg)
{
	currentDepthHeader = depthMsg->header;
	ros::WallTime start = ros::WallTime::now();
	if (!mainEngine)
		return;

	cv_bridge::CvImageConstPtr cv_depth, cv_color;
	try
	{
		cv_depth = cv_bridge::toCvShare(depthMsg);
		cv_color = cv_bridge::toCvShare(colorMsg, "rgba8");
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	ITMLib::ITMUChar4Image colorImage(ITMLib::Vector2i(cv_color->image.cols, cv_color->image.rows), true, false);
	ITMLib::ITMShortImage depthImage(ITMLib::Vector2i(cv_depth->image.cols, cv_depth->image.rows), true, false);

	cv_depth->image.convertTo(cv_depth->image, CV_16UC1);
	memcpy(depthImage.GetData(MEMORYDEVICE_CPU), cv_depth->image.data,
	       sizeof(short) * colorImage.noDims.width * colorImage.noDims.height);
	memcpy(colorImage.GetData(MEMORYDEVICE_CPU), cv_color->image.data,
	       sizeof(ITMLib::Vector4u) * colorImage.noDims.width * colorImage.noDims.height);

	if (useOdometry)
	{ // use odometry delta since last frame as pose prior for ICP
		ORUtils::SE3Pose posePrior;
		try
		{
			geometry_msgs::TransformStamped transform = tfBuffer.lookupTransform(
				depthMsg->header.frame_id, "odom", ros::Time(0));
			Eigen::Affine3d odometryPose = tf2::transformToEigen(transform);

			// compute delta to odometry from last frame
			ORUtils::SE3Pose delta(
				ORUtils::FromEigen<ITMLib::Matrix4f>((odometryPose * previousOdometryPose.inverse()).matrix()));
			posePrior = delta;
			posePrior.MultiplyWith(mainEngine->GetTrackingState()->pose_d);
			previousOdometryPose = odometryPose;
		}
		catch (tf2::TransformException& ex)
		{
			ROS_WARN("%s", ex.what());
			return;
		}
		mainEngine->ProcessFrame(&colorImage, &depthImage, nullptr, &posePrior);
	} else
	{
		mainEngine->ProcessFrame(&colorImage, &depthImage, nullptr, nullptr);
	}
	ROS_INFO("time: %.1fms", (ros::WallTime::now() - start).toSec() * 1000);

	if (mainEngine->GetTrackingState()->trackerResult != ITMLib::ITMTrackingState::TRACKING_GOOD)
		ROS_WARN_STREAM(
			"Tracking " << ITMLib::ITMTrackingState::TrackingResultToString(mainEngine->GetTrackingState()->trackerResult));

	if (mainEngine->GetTrackingState()->trackerResult == ITMLib::ITMTrackingState::TRACKING_FAILED)
		return;

	publishImage();
	publishPointCloud();
	publishTrackingResult();
}

void TSDFMapper::CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& depthMsg,
                                    const sensor_msgs::CameraInfoConstPtr& colorMsg)
{
	if (mainEngine)
		return;
	InitEngine(depthMsg, colorMsg);

	depthCameraInfoSubscriber.unsubscribe();
	colorCameraInfoSubscriber.unsubscribe();
}

void TSDFMapper::publishImage()
{

	if (renderPublisher.getNumSubscribers() <= 0)
		return;

	ITMLib::ITMUChar4Image renderedView(mainEngine->GetView()->depth->noDims, true, true);
	mainEngine->GetImage(&renderedView, ITMLib::ITMMainEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME,
	                     nullptr, nullptr, false);
	cv::Mat cvImg;
	ITMToCV(renderedView, cvImg);

	std_msgs::Header header = currentDepthHeader;
	cv_bridge::CvImage img(
		header,
		sensor_msgs::image_encodings::RGBA8,
		cvImg
	);
	sensor_msgs::ImagePtr msg = img.toImageMsg();

	renderPublisher.publish(msg);
}

void TSDFMapper::publishPointCloud()
{

	if (pointCloudPublisher.getNumSubscribers() <= 0)
		return;

	if (!pointCloud)
		pointCloud = new ITMLib::ITMPointCloud(mainEngine->GetView()->depth->noDims, MEMORYDEVICE_CUDA);
	mainEngine->GetPointCloud(pointCloud, mainEngine->GetTrackingState()->pose_d,
	                          &mainEngine->GetView()->calib.intrinsics_d, true);
	pointCloud->locations->UpdateHostFromDevice();

	const int width = mainEngine->GetView()->depth->noDims.width;
	const int height = mainEngine->GetView()->depth->noDims.height;

	ITMLib::Vector4f* points = pointCloud->locations->GetData(MEMORYDEVICE_CPU);

	sensor_msgs::PointCloud2 msg;
	if (pointCloudWithColor)
	{
		pointCloud->colours->UpdateHostFromDevice();
		ITMLib::Vector4f* colors = pointCloud->colours->GetData(MEMORYDEVICE_CPU);

		typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
		PointCloud::Ptr cloud(new PointCloud);
		cloud->height = height;
		cloud->width = width;
		for (size_t i = 0; i < width * height; i++)
		{
			pcl::PointXYZRGB point;
			point.x = points[i].x;
			point.y = points[i].y;
			point.z = points[i].z;
			point.r = static_cast<uint8_t>(255 * colors[i].r);
			point.g = static_cast<uint8_t>(255 * colors[i].g);
			point.b = static_cast<uint8_t>(255 * colors[i].b);
			cloud->points.push_back(point);
		}
		pcl::toROSMsg(*cloud, msg);
	} else
	{
		typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
		PointCloud::Ptr cloud(new PointCloud);
		cloud->height = height;
		cloud->width = width;
		for (size_t i = 0; i < width * height; i++)
		{
			cloud->points.emplace_back(pcl::PointXYZ(points[i].x, points[i].y, points[i].z));
		}
		pcl::toROSMsg(*cloud, msg);
	}


	msg.header = currentDepthHeader;
	msg.header.frame_id = globalFrameId;

	pointCloudPublisher.publish(msg);
}

void TSDFMapper::publishTrackingResult()
{
	if (!broadcastTf)
		return;

	Eigen::Matrix4d T_map_cam = ORUtils::ToEigen<Eigen::Matrix4d>(mainEngine->GetTrackingState()->pose_d->GetInvM());
	Eigen::Matrix4d T_cam_odom;
	try
	{
		geometry_msgs::TransformStamped transform = tfBuffer.lookupTransform(
			currentDepthHeader.frame_id, tfChildFrameId, ros::Time(0));
		T_cam_odom = tf2::transformToEigen(transform).matrix();
	}
	catch (tf2::TransformException& ex)
	{
		ROS_WARN("%s", ex.what());
		return;
	}

	Eigen::Matrix4d T_map_odom = T_map_cam * T_cam_odom;

	geometry_msgs::TransformStamped transform;
	transform.header = currentDepthHeader;
	transform.header.frame_id = globalFrameId;
	transform.child_frame_id = odomFrameId;

	Eigen::Vector3d translation = T_map_odom.col(3).head<3>();
	Eigen::Quaterniond quaternion(T_map_odom.block<3, 3>(0, 0));

	transform.transform.translation.x = translation[0];
	transform.transform.translation.y = translation[1];
	transform.transform.translation.z = translation[2];
	transform.transform.rotation.x = quaternion.x();
	transform.transform.rotation.y = quaternion.y();
	transform.transform.rotation.z = quaternion.z();
	transform.transform.rotation.w = quaternion.w();

	tfBroadcaster.sendTransform(transform);
}