#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include "tracetools_benchmark/tracetools.h"
#include "a6_depthimage_to_laserscan/disparity_to_depthimage_component.hpp"
#include <rclcpp/serialization.hpp>

#include "rclcpp_components/register_node_macro.hpp"

namespace robotperf
{

namespace perception
{

DisparityToDepthImageComponent::DisparityToDepthImageComponent(const rclcpp::NodeOptions &options)
: rclcpp::Node("DisparityToDepthImageComponent", options)
{
  // Get the disparity_topic_name parameter from the parameter server
  std::string disparity_topic_name = this->declare_parameter<std::string>("disparity_topic_name", "/benchmark/disparity");
  // Get the camera_info parameter from the parameter server
  std::string disparity_camera_info = this->declare_parameter<std::string>("disparity_camera_info", "/disparity_camera_info");

  // Create a custom QoS for publishers that is ROS2 default
  rclcpp::QoS custom_qos(rclcpp::KeepLast(10));
  custom_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  // Create a publisher for the Depth image
  pub_depth_ = this->create_publisher<sensor_msgs::msg::Image>("/depth", custom_qos);
  // Create a publisher for the Depth Image Camera Info
  pub_depth_camera_info_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/depth_camera_info", custom_qos);

  //Create subscriber to Disparity image and camera info with callback function
  sub_disparity_ = std::make_shared<DisparitySub>(this, disparity_topic_name);
  sub_camera_info_ = std::make_shared<CameraInfoSub>(this, disparity_camera_info);

  // Initialize sync policy to make sure we publish depth image and a camera info in sync with each other
  sync_ = std::make_shared<Sync>(SyncPolicy(10), *sub_disparity_, *sub_camera_info_);
  sync_->registerCallback(&DisparityToDepthImageComponent::disparityToDepthImageCb, this);

}


// Convert a Disparity Image to a Depth Image
void DisparityToDepthImageComponent::disparityToDepthImageCb(
  const stereo_msgs::msg::DisparityImage::SharedPtr disparity_msg,
  const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg)
{
    // Convert ROS Image to an OpenCV Message
    cv_bridge::CvImagePtr disparity_cv_ptr = cv_bridge::toCvCopy(disparity_msg->image, sensor_msgs::image_encodings::TYPE_32FC1);
    cv::Mat disparity_image = disparity_cv_ptr->image;

    // Perform disparity to depth conversion
    double baseline = disparity_msg->t; //distance between the stereo cameras in meters
    double focal_length = disparity_msg->f; //focal length 
    cv::Mat depth_image;
    depth_image = (baseline * focal_length) / (disparity_image + 1e-6); // Avoid division by zero

    // Create a new message for depth image
    cv_bridge::CvImage depth_msg; 
    depth_msg.header = disparity_msg->header; 
    depth_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; 
    depth_msg.image = depth_image; 

    // Publish the Depth Image
    pub_depth_->publish(*depth_msg.toImageMsg());
    // Publish the Depth Image Camera Info Message
    pub_depth_camera_info_->publish(*camera_info_msg);


}

}  // namespace perception

}  // namespace robotperf

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the
// component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::perception::DisparityToDepthImageComponent)
