#ifndef ROBOTPERF_LASERSCAN_OUTPUT_COMPONENT_HPP_
#define ROBOTPERF_LASERSCAN_OUTPUT_COMPONENT_HPP_

// Include C++ Libraries and ROS2 Packages
#include <ament_index_cpp/get_resource.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>


namespace robotperf
{

namespace perception
{

class LaserscanOutputComponent : public rclcpp::Node
{
    public:
        // Constructor
        explicit LaserscanOutputComponent(const rclcpp::NodeOptions &options);

    private:
        // Member variable for subscriber to the laser scan messages
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_sub_;

        // Member function to get size of message
        size_t get_msg_size(sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);
        // Member function to handle received LaserScan messages
        void laserscanCb(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

};

} // namespace perception
    
} // namespace robotperf

#endif

