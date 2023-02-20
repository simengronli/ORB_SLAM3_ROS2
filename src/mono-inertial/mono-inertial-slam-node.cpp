#include "mono-inertial-slam-node.hpp"

#include<opencv2/core/core.hpp>




using std::placeholders::_1;

MonoInertialSlamNode::MonoInertialSlamNode(ORB_SLAM3::System* pSLAM)
:   Node("ORB_SLAM3_ROS2")
{
    m_SLAM = pSLAM;
    // std::cout << "slam changed" << std::endl;
    m_image_subscriber = this->create_subscription<ImageMsg>(
        "camera",
        100,
        std::bind(&MonoInertialSlamNode::GrabImage, this, std::placeholders::_1));
    std::cout << "slam changed" << std::endl;

    m_imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu",
        1000,
        std::bind(&MonoInertialSlamNode::GrabImu, this, std::placeholders::_1));


    // Create a tf broadcaster to broadcast the camera pose
    m_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    // create a static tf broadcaster to broadcast the telloBase_link to camera_link
    m_static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

    syncThread = new std::thread(&MonoInertialSlamNode::SyncWithImu, this);
}

MonoInertialSlamNode::~MonoInertialSlamNode()
{
    // Stop all threads
    m_SLAM->Shutdown();

    // Save camera trajectory
    m_SLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void StereoInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    mutexImuQueue.lock();
    imu_queue.push(msg);
    mutexImuQueue.unlock();
}

void StereoInertialNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    mutexImageQueue.lock();

    if (!image_queue.empty())
        image_queue.pop();
    image_queue.push(msg);

    mutexImageQueue.unlock();
}

cv::Mat MonoInertialSlamNode::GetImage(const ImageMsg::SharedPtr msg){
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }  
}



void MonoInertialSlamNode::SyncWithIMU(const ImageMsg::SharedPtr msg)
{
    while (1)
    {
        cv::Mat img;
        double tImg = 0;
        if (!image_queue.empty() && !imu_queue.empty())
        {
            tImg = Utility::StampToSec(image_queue.front()->header.stamp);

        
            mutexImageQueue.lock();
            img = GetImage(image_queue.front());
            image_queue.pop();
            mutexImageQueue.unlock();

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            mutexImuQueue.lock();
            if (!imu_queue.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!imu_queue.empty() && Utility::StampToSec(imu_queue.front()->header.stamp) <= tImg)
                {
                    double t = Utility::StampToSec(imu_queue.front()->header.stamp);
                    cv::Point3f acc(imu_queue.front()->linear_acceleration.x, imu_queue.front()->linear_acceleration.y, imu_queue.front()->linear_acceleration.z);
                    cv::Point3f gyr(imu_queue.front()->angular_velocity.x, imu_queue.front()->angular_velocity.y, imu_queue.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imu_queue.pop();
                }
            }
            mutexImuQueue.unlock();

            Sophus::SE3f Tcw = m_SLAM->TrackMonocular(m_cvImPtr->image, Utility::StampToSec(msg->header.stamp), vImuMeas);
            this->BroadcastCameraTransform(Tcw.inverse());
            
            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}

void MonoInertialSlamNode::BroadcastCameraTransform(Sophus::SE3f Tcw)
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
