#ifndef FLIGHT_PANEL_HPP
#define FLIGHT_PANEL_HPP

#include <rviz_common/panel.hpp>
#include <QtWidgets>
#include <QComboBox> 
#include <QTimer>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <string>
#include <std_msgs/msg/int32.hpp>

namespace insertion_base_station {

// =====================================================================
// HUD WIDGET CLASS
// =====================================================================
class HUDWidget : public QWidget {
    Q_OBJECT
public:
    explicit HUDWidget(QWidget* parent = nullptr);
    void setFlightData(double roll, double pitch, double heading, double altitude);
    void setTelemetryStatus(bool has_telemetry); 

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    double roll_deg_ = 0.0;
    double pitch_deg_ = 0.0;
    double heading_deg_ = 0.0;
    double altitude_ = 0.0;
    
    bool has_telemetry_ = false; // Tracks connection status
};

// =====================================================================
// MAIN RVIZ PANEL CLASS
// =====================================================================
class FlightPanel : public rviz_common::Panel {
    Q_OBJECT
public:
    explicit FlightPanel(QWidget* parent = nullptr);
    virtual void onInitialize() override;

protected Q_SLOTS:
    void updateFlightData();
    void onDroneChanged(int index); 

private:
    HUDWidget* hud_widget_;
    QComboBox* drone_selector_cb_; 
    QTimer* update_timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string map_frame_ = "map";
    std::string drone_frame_ = "drone1/base_link"; // Default target

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr focus_pub_;
    
    double tf_timeout_s_;

    // Safety Watchdog Variables
    int tf_fail_count_ = 0;
    const int MAX_TF_FAILS = 15; // At 33ms per tick, 15 fails = ~0.5 seconds without data
};

} // namespace insertion_base_station

#endif // FLIGHT_PANEL_HPP