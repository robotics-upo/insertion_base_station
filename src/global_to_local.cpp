#include <memory>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <GeographicLib/LocalCartesian.hpp>

using namespace std::chrono_literals;

class GlobalToLocal : public rclcpp::Node {
public:
    GlobalToLocal()
    : Node("global_to_local"),
        origin_set_(false)
    {
        map_frame_ = this->declare_parameter<std::string>("global_frame", "map");
        base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");

        // subscribe to NavSatFix
        auto qos = rclcpp::SensorDataQoS();
        sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "fix", qos,
            std::bind(&GlobalToLocal::fixCallback, this, std::placeholders::_1));

        sub2_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "gps", qos,
            std::bind(&GlobalToLocal::gpsCallback, this, std::placeholders::_1));


        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(this->get_logger(), "global_to_local node started, publishing transforms %s -> %s",
                                map_frame_.c_str(), base_frame_.c_str());
    } 

private:
    void fixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (std::isnan(msg->latitude) || std::isnan(msg->longitude) || std::isnan(msg->altitude)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received invalid GPS fix");
            return;
        }

        if (!origin_set_) {
            lat0_ = msg->latitude;
            lon0_ = msg->longitude;
            alt0_ = msg->altitude;
            
            local_cart_ = std::make_unique<GeographicLib::LocalCartesian>(lat0_, lon0_, alt0_);
            origin_set_ = true;
            RCLCPP_INFO(this->get_logger(), "Origin set to lat=%f lon=%f alt=%f", lat0_, lon0_, alt0_);
        }

    }

    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (std::isnan(msg->latitude) || std::isnan(msg->longitude) || std::isnan(msg->altitude)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Received invalid GPS fix");
            return;
        }

        if (origin_set_) {
            
            double x, y, z;
            // convert global (lat, lon, alt) -> local (x east, y north, z up)
            local_cart_->Forward(msg->latitude, msg->longitude, msg->altitude, x, y, z);

            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = this->now();
            t.header.frame_id = map_frame_;
            t.child_frame_id = base_frame_;
            t.transform.translation.x = x;
            t.transform.translation.y = y;
            t.transform.translation.z = z;
            // no orientation from GPS alone; keep identity
            t.transform.rotation.x = 0.0;
            t.transform.rotation.y = 0.0;
            t.transform.rotation.z = 0.0;
            t.transform.rotation.w = 1.0;

            tf_broadcaster_->sendTransform(t);
        }
    }


    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_, sub2_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<GeographicLib::LocalCartesian> local_cart_;

    bool origin_set_;
    double lat0_, lon0_, alt0_;
    std::string map_frame_;
    std::string base_frame_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalToLocal>());
    rclcpp::shutdown();
    return 0;
}