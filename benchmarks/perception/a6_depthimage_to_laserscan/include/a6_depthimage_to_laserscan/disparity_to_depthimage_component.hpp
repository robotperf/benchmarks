#ifndef ROBOTPERF_DISPARITY_TO_DEPTHIMAGE_HPP_
#define ROBOTPERF_DISPARITY_TO_DEPTHIMAGE_HPP_

#include <ament_index_cpp/get_resource.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace robotperf
{

namespace perception
{

class DisparityToDepthImageComponent
  : public rclcpp::Node
{
public:
  explicit DisparityToDepthImageComponent(const rclcpp::NodeOptions &options);

protected:
  // Create subscriber to Disparity image with callback function
  // rclcpp::Subscription<stereo_msgs::msg::DisparityImage>::SharedPtr sub_disparity_;

  void disparityToDepthImageCb(
    const stereo_msgs::msg::DisparityImage::SharedPtr disparity_msg, 
    const sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg);

  // Defining the message filters and sync policy
  typedef message_filters::Subscriber<stereo_msgs::msg::DisparityImage> DisparitySub;
  typedef message_filters::Subscriber<sensor_msgs::msg::CameraInfo> CameraInfoSub;
  typedef message_filters::sync_policies::ApproximateTime<stereo_msgs::msg::DisparityImage, sensor_msgs::msg::CameraInfo> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;

  // Subscribers for disparity image and camera info
  std::shared_ptr<DisparitySub> sub_disparity_;
  std::shared_ptr<CameraInfoSub> sub_camera_info_;
  std::shared_ptr<Sync> sync_;


  // Create publisher of a depth image
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_;
  // Create publisher for depth image camera info
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_depth_camera_info_; 


};

}  // namespace perception

}  // namespace robotperf

#endif 