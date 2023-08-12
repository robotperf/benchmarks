// Include C++ Libraries and ROS2 Packages
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// Include custom header files
#include "tracetools_benchmark/tracetools.h"
#include "a6_depthimage_to_laserscan/laserscan_output_component.hpp"
#include <rclcpp/serialization.hpp>

// Include macro to register the node as a component with class_loader
#include "rclcpp_components/register_node_macro.hpp"

// Define namespace for the component
namespace robotperf
{

namespace perception
{
    // Define the LaserscanOutputComponent class which inherits from rclcpp::Node
    LaserscanOutputComponent::LaserscanOutputComponent(const rclcpp::NodeOptions & options)
    : rclcpp::Node("LaserscanOutputComponent", options)
    {
        // Get the output topic name paramter from the parameter server with default value "/scan"
        std::string output_topic_name = this->declare_parameter<std::string>("output_topic_name","/scan");

        // Create a subscriber to listen to the laserscan messages
        // KeepLast(10) specifies subscriber stores last 10 messages received in case of disconnections
        // reliable() subscriber receives all the messages in order and without duplicates
        // best_effort() if subscriber can't keep up with messages being published, it will skip any messages it misses
        laserscan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            output_topic_name, 
            rclcpp::QoS(rclcpp::KeepLast(10)).reliable().best_effort(), 
            std::bind(&LaserscanOutputComponent::laserscanCb, this, std::placeholders::_1)
        );


    }

    size_t LaserscanOutputComponent::get_msg_size(sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg){
        // Create SerializedMessage object which is a ROS 2 message type that can hold a serialized message
        rclcpp::SerializedMessage serialized_data_scan;
        // Create Serialization object that is used to serialize a sensor_msgs::msg::LaserScan message
        rclcpp::Serialization<sensor_msgs::msg::LaserScan> scan_serialization;
        // Use reinterpret_cast to cast shared pointer to a const void pointer and create a pointer to scan_msg LaserScan message.
        const void* scan_ptr = reinterpret_cast<const void*>(scan_msg.get());
        // Serialize the LaserScan message into the SerializedMessage object
        scan_serialization.serialize_message(scan_ptr, &serialized_data_scan);
        // Using the Serialization object, serialize the LaserScan message into the SerializedMessage object
        size_t scan_msg_size = serialized_data_scan.size();
        // Return the size of the serialized message
        return scan_msg_size;
    }


    //Member function to handle the received LaserScan messages
    void LaserscanOutputComponent::laserscanCb(
        const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        TRACEPOINT(
            robotperf_laserscan_output_cb_init,
            static_cast<const void *>(this),
            static_cast<const void *>(&(*scan_msg)),
            scan_msg->header.stamp.nanosec,
            scan_msg->header.stamp.sec,
            get_msg_size(scan_msg));

        TRACEPOINT(
            robotperf_laserscan_output_cb_fini,    
            static_cast<const void *>(this),
            static_cast<const void *>(&(*scan_msg)),
            scan_msg->header.stamp.nanosec,
            scan_msg->header.stamp.sec,
            get_msg_size(scan_msg));

    }
} // namespace perception

} // namespace robotperf

// Register component with class_loader
// Allows component to be discoverable when
// library is loaded to a running process
RCLCPP_COMPONENTS_REGISTER_NODE(robotperf::perception::LaserscanOutputComponent);