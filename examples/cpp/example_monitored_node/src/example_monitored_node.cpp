/**
 * @file example_monitored_node.cpp
 * @brief Example ROS2 C++ node that registers itself with ros2top for monitoring
 * 
 * This example demonstrates:
 * 1. How to register a ROS2 node with ros2top using C++
 * 2. How to send periodic heartbeats
 * 3. How to unregister on shutdown
 * 4. Basic ROS2 node functionality with publishing and subscribing
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

// Include ros2top registry functions
#include <ros2top/ros2top.hpp>

using namespace std::chrono_literals;

class ExampleMonitoredNode : public rclcpp::Node
{
public:
    ExampleMonitoredNode() : Node("example_monitored_node_cpp"), message_count_(0), ros2top_registered_(false)
    {
        // Register with ros2top
        register_with_ros2top();
        
        // Create publisher and subscriber for demonstration
        publisher_ = this->create_publisher<std_msgs::msg::String>("example_topic_cpp", 10);
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "example_input_cpp", 10,
            std::bind(&ExampleMonitoredNode::listener_callback, this, std::placeholders::_1));
        
        // Create timer for periodic publishing
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&ExampleMonitoredNode::timer_callback, this));
            
        // Create timer for periodic heartbeat
        heartbeat_timer_ = this->create_wall_timer(
            5000ms, std::bind(&ExampleMonitoredNode::heartbeat_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "Example monitored C++ node started");
        RCLCPP_INFO(this->get_logger(), "ros2top monitoring: %s", 
                   ros2top_registered_ ? "enabled" : "disabled");
    }
    
    ~ExampleMonitoredNode()
    {
        shutdown_node();
    }

private:
    void register_with_ros2top()
    {
        try {
            // Create node information JSON
            nlohmann::json node_info;
            node_info["description"] = "Example ROS2 C++ node demonstrating ros2top integration";
            node_info["version"] = "1.0.0";
            node_info["topics_published"] = nlohmann::json::array({"example_topic_cpp"});
            node_info["topics_subscribed"] = nlohmann::json::array({"example_input_cpp"});
            node_info["node_type"] = "example_demo_cpp";
            node_info["language"] = "cpp";
            
            // Register with ros2top
            bool success = ros2top::register_node(this->get_name(), node_info);
            if (success) {
                ros2top_registered_ = true;
                RCLCPP_INFO(this->get_logger(), "Successfully registered with ros2top");
            } else {
                RCLCPP_WARN(this->get_logger(), "Failed to register with ros2top");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error registering with ros2top: %s", e.what());
        }
    }
    
    void timer_callback()
    {
        // Create and publish a message
        auto message = std_msgs::msg::String();
        message.data = "Hello from monitored C++ node! Message #" + std::to_string(message_count_);
        publisher_->publish(message);
        
        RCLCPP_INFO(this->get_logger(), "Published: %s", message.data.c_str());
        message_count_++;
        
        // Simulate some CPU work
        auto start_time = std::chrono::high_resolution_clock::now();
        while (std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::high_resolution_clock::now() - start_time).count() < 100) {
            // 100ms of work
            volatile int sum = 0;
            for (int i = 0; i < 1000; ++i) {
                sum += i * i;
            }
        }
    }
    
    void heartbeat_callback()
    {
        if (ros2top_registered_) {
            try {
                bool success = ros2top::heartbeat(this->get_name());
                if (success) {
                    RCLCPP_DEBUG(this->get_logger(), "Sent heartbeat to ros2top");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Failed to send heartbeat to ros2top");
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error sending heartbeat: %s", e.what());
            }
        }
    }
    
    void listener_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received: %s", msg->data.c_str());
        
        // Simulate some processing work
        std::this_thread::sleep_for(50ms); // 50ms processing time
    }
    
    void shutdown_node()
    {
        if (ros2top_registered_) {
            try {
                bool success = ros2top::unregister_node(this->get_name());
                if (success) {
                    RCLCPP_INFO(this->get_logger(), "Successfully unregistered from ros2top");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Failed to unregister from ros2top");
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error unregistering from ros2top: %s", e.what());
            }
        }
    }
    
    // Member variables
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    size_t message_count_;
    bool ros2top_registered_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ExampleMonitoredNode>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception in main: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
