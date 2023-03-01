#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>




using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    // std::cout << "slam changed" << std::endl;
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera",
        10,
        std::bind(&MonocularSlamNode::GrabImage, this, std::placeholders::_1));
    std::cout << "slam changed" << std::endl;

    // Create a tf broadcaster to broadcast the camera pose
    m_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // create a static tf broadcaster to broadcast the telloBase_link to camera_link
    m_static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
}

MonocularSlamNode::~MonocularSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
    try
    {
        m_cvImPtr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // std::cout<<"one frame has been sent"<<std::endl;
    // m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
    // ------------------test------------------
    Sophus::SE3f Tcw = m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
    this->BroadcastCameraTransform(Tcw);
}

void MonocularSlamNode::BroadcastCameraTransform(Sophus::SE3f Tcw)
{
    Sophus::SE3f Twc = Tcw.inverse();

    // Create a transform from the camera pose
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = rclcpp::Clock().now();
    transform_stamped.header.frame_id = "world";
    transform_stamped.child_frame_id = "telloCamera";

    // Rotate Twc -90 degrees around the x axis
    Eigen::Quaternionf q_rot = Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitX()) * Eigen::Quaternionf::Identity();
    Eigen::Quaternionf q_rotated = q_rot * Twc.unit_quaternion().cast<float>();
    transform_stamped.transform.rotation.x = q_rotated.x();
    transform_stamped.transform.rotation.y = q_rotated.y();
    transform_stamped.transform.rotation.z = q_rotated.z();
    transform_stamped.transform.rotation.w = q_rotated.w();

    // Translate Twc
    Eigen::Vector3f t_rotated = q_rot * Twc.translation().cast<float>();
    transform_stamped.transform.translation.x = t_rotated.x();
    transform_stamped.transform.translation.y = t_rotated.y();
    transform_stamped.transform.translation.z = t_rotated.z();
    


    // Create a static transform from the telloBase_link to camera_link
    geometry_msgs::msg::TransformStamped static_transform_stamped;
    static_transform_stamped.header.stamp = rclcpp::Clock().now();
    static_transform_stamped.header.frame_id = "telloCamera";
    static_transform_stamped.child_frame_id = "telloIMU";    

    // Make the transform to transform from camera_link to telloBase_link
    static_transform_stamped.transform.translation.x = 0.0;
    static_transform_stamped.transform.translation.y = -0.028;
    static_transform_stamped.transform.translation.z = -0.043;

    // Rotate the IMU -90 degrees around the y axis
    Eigen::Quaternionf q_rot_imu = Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitY()) * Eigen::Quaternionf::Identity();
    static_transform_stamped.transform.rotation.x = q_rot_imu.x();
    static_transform_stamped.transform.rotation.y = q_rot_imu.y();
    static_transform_stamped.transform.rotation.z = q_rot_imu.z();
    static_transform_stamped.transform.rotation.w = q_rot_imu.w();


    // Send the transform
    m_tf_broadcaster_->sendTransform(transform_stamped);
    
    // Send the static transform
    m_static_tf_broadcaster_->sendTransform(static_transform_stamped);
}
