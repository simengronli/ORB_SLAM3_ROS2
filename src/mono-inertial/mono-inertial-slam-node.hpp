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

using ImageMsg = sensor_msgs::msg::Image;
using ImuMsg = sensor_msgs::msg::Imu;

class MonoInertialSlamNode : public rclcpp::Node
{
public:
    MonoInertialSlamNode(ORB_SLAM3::System* pSLAM);

    ~MonoInertialSlamNode();

private:

    void GrabImage(const ImageMsg::SharedPtr msg);
    cv::Mat GetImage(const ImageMsg::SharedPtr msg);
    void GrabImu(const ImuMsg::SharedPtr msg);
    void SyncWithImu();
    void GrabMonoInertial(const ImageMsg::SharedPtr msgImage, const ImuMsg::SharedPtr msgImu);

    // rclcpp::Subscription<ImageMsg>::SharedPtr m_image_subscriber;
    // rclcpp::Subscription<ImuMsg>::SharedPtr m_imu_subscriber;

    // message filter m_sync
    typedef message_filters::sync_policies::ApproximateTime<ImageMsg, ImuMsg> approximate_sync_policy;

    std::shared_ptr<message_filters::Subscriber<ImageMsg>> image_subscriber;
    std::shared_ptr<message_filters::Subscriber<ImuMsg>> imu_subscriber;
    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> syncApproximate;

    queue<ImuMsg::SharedPtr> imu_queue;
    std::mutex mutexImuQueue;
    queue<ImageMsg::SharedPtr> image_queue;
    std::mutex mutexImageQueue;


    ORB_SLAM3::System *m_SLAM;
    std::thread *syncThread;

    // broadcast the camera pose tf
    void BroadcastCameraTransform(Sophus::SE3f Tcw);
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster_;

    // static telloBase_link to camera_link tf
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_static_tf_broadcaster_;




};

#endif
