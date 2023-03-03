#ifndef __MONO_INERTIAL_SLAM_NODE_HPP__
#define __MONO_INERTIAL_SLAM_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

// include message_filters and sync_policies
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"


class MonoInertialSlamNode : public rclcpp::Node
{
public:
    MonoInertialSlamNode(ORB_SLAM3::System* pSLAM);

    ~MonoInertialSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    using ImuMsg = sensor_msgs::msg::Imu;
    typedef message_filters::sync_policies::ApproximateTime<ImageMsg, ImuMsg> approximate_sync_policy;
    
    void GrabMonoInertial(const ImageMsg::SharedPtr msgImage, const ImuMsg::SharedPtr msgImu);

    ORB_SLAM3::System *m_SLAM;

    cv_bridge::CvImageConstPtr cv_ptr;

    std::shared_ptr<message_filters::Subscriber<ImageMsg>> image_subscriber;
    std::shared_ptr<message_filters::Subscriber<ImuMsg>> imu_subscriber;

    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> syncApproximate;
    
    void BroadcastCameraTransform(Sophus::SE3f Tcw);
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_static_tf_broadcaster_;
};

#endif
