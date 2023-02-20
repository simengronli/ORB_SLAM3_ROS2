#include "mono-inertial-slam-node.hpp"

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

    std::cout<<"one frame has been sent"<<std::endl;
    // m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
    // ------------------test------------------
    Sophus::SE3f Tcw = m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp));
    this->BroadcastCameraTransform(Tcw.inverse());
}

void MonocularSlamNode::BroadcastCameraTransform(Sophus::SE3f Tcw)
{
    // Create a transform from the camera pose
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = rclcpp::Clock().now();
    transform_stamped.header.frame_id = "world";
    transform_stamped.child_frame_id = "telloCamera";

    transform_stamped.transform.translation.x = Tcw.matrix()(0,3);
    transform_stamped.transform.translation.y = Tcw.matrix()(1,3);
    transform_stamped.transform.translation.z = Tcw.matrix()(2,3);

    transform_stamped.transform.rotation.x = Tcw.unit_quaternion().x();
    transform_stamped.transform.rotation.y = Tcw.unit_quaternion().y();
    transform_stamped.transform.rotation.z = Tcw.unit_quaternion().z();
    transform_stamped.transform.rotation.w = Tcw.unit_quaternion().w();


    // Create a static transform from the telloBase_link to camera_link
    geometry_msgs::msg::TransformStamped static_transform_stamped;
    static_transform_stamped.header.stamp = rclcpp::Clock().now();
    static_transform_stamped.header.frame_id = "telloCamera";
    static_transform_stamped.child_frame_id = "telloBase";    

    // Make the transform to transform from camera_link to telloBase_link
    static_transform_stamped.transform.translation.x = 0.0;
    static_transform_stamped.transform.translation.y = 0.0;
    static_transform_stamped.transform.translation.z = 0.0;

    static_transform_stamped.transform.rotation.x = 0.5;
    static_transform_stamped.transform.rotation.y = -0.5;
    static_transform_stamped.transform.rotation.z = 0.5;
    static_transform_stamped.transform.rotation.w = 0.5;

    // Send the transform
    m_tf_broadcaster_->sendTransform(transform_stamped);
    
    // Send the static transform
    m_static_tf_broadcaster_->sendTransform(static_transform_stamped);
}
