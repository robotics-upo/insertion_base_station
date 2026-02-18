#include <memory>
#include <chrono>
#include <cmath>
#include <vector>
#include <string>
#include <deque> 

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp" 
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <GeographicLib/LocalCartesian.hpp>

using namespace std::chrono_literals;

/*
 * GlobalToLocal Node
 * ------------------
 * Converts global GNSS coordinates (Lat/Lon/Alt) into a local Cartesian ENU (East-North-Up) frame.
 */
class GlobalToLocal : public rclcpp::Node {
public:
    GlobalToLocal()
    : Node("global_to_local"),
      origin_set_(false),
      has_orientation_(false),
      initial_z_(-1e10)
    {
        // --- PARAMETER CONFIGURATION ---
        
        // Frames
        map_frame_ = this->declare_parameter<std::string>("global_frame", "map");
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
        
        // Topics
        std::string path_topic = this->declare_parameter<std::string>("path_topic", "path");
        std::string origin_topic = "fix"; // Topic used to set the (0,0,0) reference
        std::string rover_topic = "gps";  // Topic used for the moving vehicle

        // Settings
        this->declare_parameter("use_z", true);
        this->declare_parameter("relative_z", true);
        this->declare_parameter("z_offset", 0.0);
        this->declare_parameter("max_path_size", 5000); 

        use_z_ = this->get_parameter("use_z").as_bool();
        relative_z_ = this->get_parameter("relative_z").as_bool();
        z_offset_ = this->get_parameter("z_offset").as_double();
        max_path_size_ = this->get_parameter("max_path_size").as_int();

        // QoS Profile
        auto qos = rclcpp::SensorDataQoS();

        // --- SUBSCRIBERS ---

        // 1. Origin Listener
        sub_origin_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            origin_topic, qos,
            std::bind(&GlobalToLocal::setOriginCallback, this, std::placeholders::_1));

        // 2. Rover Listener
        sub_rover_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            rover_topic, qos,
            std::bind(&GlobalToLocal::updatePositionCallback, this, std::placeholders::_1));

        // 3. Attitude Listener
        sub_att_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
            "/dji_sdk/attitude", qos,
            std::bind(&GlobalToLocal::attitudeCallback, this, std::placeholders::_1));

        // --- PUBLISHERS ---
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_topic, 10);

        // Initialize orientation to Identity
        current_orientation_.w = 1.0;

        RCLCPP_INFO(this->get_logger(), "GlobalToLocal Node Initialized.");
        RCLCPP_INFO(this->get_logger(), "Waiting for Origin Fix on topic: '%s'...", origin_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing Path to: '%s'", path_topic.c_str());
    } 

private:
    // Stores the latest orientation data.
    void attitudeCallback(const geometry_msgs::msg::QuaternionStamped::SharedPtr msg)
    {
        current_orientation_ = msg->quaternion;
        has_orientation_ = true;
    }

    // Sets the Local Tangent Plane origin (Lat0, Lon0, Alt0).
    void setOriginCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (std::isnan(msg->latitude) || std::isnan(msg->longitude)) return;

        if (!origin_set_) {
            lat0_ = msg->latitude;
            lon0_ = msg->longitude;
            alt0_ = (use_z_) ? msg->altitude : 0.0;

            local_cart_ = std::make_unique<GeographicLib::LocalCartesian>(lat0_, lon0_, alt0_);
            origin_set_ = true;
            
            RCLCPP_INFO(this->get_logger(), "Local Origin ESTABLISHED at: [%.6f, %.6f, %.2f]", lat0_, lon0_, alt0_);
        }
    }

    // Main loop. Converts Rover Global coords -> Local Cartesian -> TF & Path.
    void updatePositionCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (std::isnan(msg->latitude) || std::isnan(msg->longitude)) return;

        if (origin_set_) {
            double x, y, z;
            
            local_cart_->Forward(msg->latitude, msg->longitude, msg->altitude, x, y, z);

            // --- Z-AXIS HANDLING ---
            if (!use_z_) {
                z = 0.0;
            } else if (relative_z_) {
                if (initial_z_ < -1e9) {
                    initial_z_ = z;
                    z = 0.0;
                } else {
                    z = z - initial_z_;
                }
            }
            z += z_offset_;

            rclcpp::Time current_time = this->now();

            // --- 1. BROADCAST TF ---
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = current_time;
            t.header.frame_id = map_frame_;
            t.child_frame_id = base_frame_;
            
            t.transform.translation.x = x;
            t.transform.translation.y = y;
            t.transform.translation.z = z;

            t.transform.rotation = has_orientation_ ? current_orientation_ : geometry_msgs::msg::Quaternion().set__w(1.0);

            tf_broadcaster_->sendTransform(t);

            // --- 2. PUBLISH PATH ---
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = current_time;
            pose.header.frame_id = map_frame_;
            
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = z;
            pose.pose.orientation = t.transform.rotation;

            path_msg_.header.stamp = current_time;
            path_msg_.header.frame_id = map_frame_;
            path_msg_.poses.push_back(pose);

            if (path_msg_.poses.size() > static_cast<size_t>(max_path_size_)) {
                path_msg_.poses.erase(path_msg_.poses.begin());
            }

            path_pub_->publish(path_msg_);
        }
    }

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_origin_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_rover_;
    rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr sub_att_;
    
    // Publishers & TF
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_msg_; 
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Core Logic
    std::unique_ptr<GeographicLib::LocalCartesian> local_cart_;
    
    // State Variables (Order matters for initialization!)
    bool origin_set_;
    bool has_orientation_; // Declarado aqui
    
    // Configuration & Data
    double lat0_, lon0_, alt0_;
    double initial_z_;   // Declarado despues, por eso debe inicializarse despues
    
    // Parameters
    double z_offset_;
    int max_path_size_;
    std::string map_frame_;
    std::string base_frame_;
    bool use_z_, relative_z_;

    geometry_msgs::msg::Quaternion current_orientation_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalToLocal>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}