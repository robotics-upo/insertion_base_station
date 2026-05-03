#include "insertion_base_station/flight_panel.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <algorithm> 

namespace insertion_base_station {

// =====================================================================
// HUD DRAWING WIDGET
// =====================================================================
HUDWidget::HUDWidget(QWidget* parent) : QWidget(parent) {
    setMinimumSize(300, 200);
}

void HUDWidget::setFlightData(double roll, double pitch, double heading, double altitude) {
    roll_deg_ = roll;
    pitch_deg_ = pitch;
    heading_deg_ = heading;
    altitude_ = altitude;
    has_telemetry_ = true;
    update();
}

void HUDWidget::setTelemetryStatus(bool has_telemetry) {
    if (has_telemetry_ != has_telemetry) {
        has_telemetry_ = has_telemetry;
        update();
    }
}

void HUDWidget::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    int w = width();
    int h = height();
    int cx = w / 2;
    int cy = h / 2;

    // --- 1. DRAW ARTIFICIAL HORIZON ---
    painter.save();
    painter.translate(cx, cy);
    painter.rotate(-roll_deg_);
    int pitch_offset = static_cast<int>(pitch_deg_ * 4.0); 
    painter.translate(0, pitch_offset);

    painter.fillRect(-w*2, -h*2, w*4, h*2, QColor(0, 120, 200));  // Sky (Blue)
    painter.fillRect(-w*2, 0, w*4, h*2, QColor(110, 70, 30));     // Ground (Brown)
    
    painter.setPen(QPen(Qt::white, 2));
    painter.drawLine(-w*2, 0, w*4, 0); // Horizon line
    painter.restore();

    // --- 2. DRAW CENTER CROSSHAIR ---
    painter.setPen(QPen(Qt::yellow, 3));
    painter.drawLine(cx - 30, cy, cx - 10, cy);
    painter.drawLine(cx + 10, cy, cx + 30, cy);
    painter.drawLine(cx, cy - 10, cx, cy + 10);

    // --- 3. DRAW ALTITUDE BAR ---
    int bar_w = 20;
    int bar_h = h - 40;
    int bar_x = 20;
    int bar_y = 20;

    painter.fillRect(bar_x, bar_y, bar_w, bar_h, QColor(0, 0, 0, 150));

    double min_alt = -5.0;
    double max_alt = 30.0;
    double range = max_alt - min_alt;
    double alt_clamped = std::clamp(altitude_, min_alt, max_alt);
    double fill_ratio = (alt_clamped - min_alt) / range;
    int fill_h = static_cast<int>(fill_ratio * bar_h);

    painter.fillRect(bar_x, bar_y + bar_h - fill_h, bar_w, fill_h, QColor(0, 255, 0, 200));

    painter.setPen(Qt::white);
    painter.drawText(bar_x, bar_y - 20, bar_w * 2, 20, Qt::AlignLeft | Qt::AlignVCenter, QString::number(altitude_, 'f', 1) + "m");

    // --- 4. DRAW COMPASS ---
    int comp_radius = 35;
    int comp_x = w - 60;
    int comp_y = cy;

    painter.setBrush(QColor(0, 0, 0, 150));
    painter.setPen(Qt::NoPen);
    painter.drawEllipse(QPoint(comp_x, comp_y), comp_radius, comp_radius);

    painter.save();
    painter.translate(comp_x, comp_y);
    painter.rotate(-heading_deg_); 

    painter.setPen(Qt::white);
    QFont font = painter.font();
    font.setBold(true);
    painter.setFont(font);

    painter.drawText(-10, -comp_radius + 2, 20, 15, Qt::AlignCenter, "N");
    painter.drawText(comp_radius - 15, -7, 15, 15, Qt::AlignCenter, "E");
    painter.drawText(-10, comp_radius - 15, 20, 15, Qt::AlignCenter, "S");
    painter.drawText(-comp_radius + 2, -7, 15, 15, Qt::AlignCenter, "W");
    painter.restore();

    // Compass indicator (Red Triangle)
    painter.setBrush(Qt::red);
    painter.setPen(Qt::red);
    QPoint indicator[3] = {
        QPoint(comp_x, comp_y - comp_radius - 5),
        QPoint(comp_x - 5, comp_y - comp_radius + 5),
        QPoint(comp_x + 5, comp_y - comp_radius + 5)
    };
    painter.drawPolygon(indicator, 3);

    // --- 5. SAFETY OVERLAY (NO TELEMETRY) ---
    if (!has_telemetry_) {
        painter.fillRect(0, 0, w, h, QColor(0, 0, 0, 150));
        painter.setPen(Qt::red);
        QFont warn_font = painter.font();
        warn_font.setPointSize(16);
        warn_font.setBold(true);
        painter.setFont(warn_font);
        painter.drawText(0, 0, w, h, Qt::AlignCenter, "NO TELEMETRY\n(TF LOST)");
    }
}

// =====================================================================
// ROS LOGIC & PANEL UI
// =====================================================================
FlightPanel::FlightPanel(QWidget* parent) : rviz_common::Panel(parent) {
    QVBoxLayout* layout = new QVBoxLayout;

    drone_selector_cb_ = new QComboBox(this);
    drone_selector_cb_->addItem("Telemetry: Drone 1");
    drone_selector_cb_->addItem("Telemetry: Drone 2");
    
    // Modern Qt connect syntax
    connect(drone_selector_cb_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &FlightPanel::onDroneChanged);
    
    layout->addWidget(drone_selector_cb_);

    hud_widget_ = new HUDWidget(this);
    layout->addWidget(hud_widget_);
    layout->setContentsMargins(0,0,0,0); 
    setLayout(layout);

    update_timer_ = new QTimer(this);
    // Modern Qt connect syntax
    connect(update_timer_, &QTimer::timeout, this, &FlightPanel::updateFlightData);
}

void FlightPanel::onInitialize() {
    auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    
    node->declare_parameter("tf_timeout_s", 2.0);
    tf_timeout_s_ = node->get_parameter("tf_timeout_s").as_double();

    focus_pub_ = node->create_publisher<std_msgs::msg::Int32>("/gui/focus_drone", 10);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Start UI loop at 100ms (~10 FPS)
    update_timer_->start(100); 
}

void FlightPanel::onDroneChanged(int index) {
    drone_frame_ = (index == 0) ? "drone1/base_link" : "drone2/base_link";

    if (focus_pub_) {
        std_msgs::msg::Int32 msg;
        msg.data = index;
        focus_pub_->publish(msg);
    }
    
    // Reset watchdog on switch to prevent instant errors
    tf_fail_count_ = 0; 
}

void FlightPanel::updateFlightData() {
    if (!tf_buffer_) return;

    try {
        auto transform = tf_buffer_->lookupTransform(map_frame_, drone_frame_, tf2::TimePointZero);

        // --- VALIDATION OF TF AGE ---
        auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
        rclcpp::Time tf_time(transform.header.stamp);
        rclcpp::Time current_time = node->now();
        
        // If the last TF is older than timeout, we force the disconnection error
        if ((current_time - tf_time).seconds() > tf_timeout_s_) {
            throw tf2::TransformException("TF is too old (signal frozen)");
        }
        // -----------------------------------------

        double z_altitude = transform.transform.translation.z;

        tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w);
        
        tf2::Matrix3x3 m(q);
        double roll_rad, pitch_rad, yaw_rad;
        m.getRPY(roll_rad, pitch_rad, yaw_rad);

        // Convert radians to degrees
        double roll_deg = roll_rad * 180.0 / M_PI;
        double pitch_deg = pitch_rad * 180.0 / M_PI;
        double heading_deg = yaw_rad * 180.0 / M_PI;
        if (heading_deg < 0) heading_deg += 360.0;

        // Feed data to HUD
        hud_widget_->setFlightData(roll_deg, -pitch_deg, heading_deg, z_altitude);
        
        // Reset safety watchdog
        tf_fail_count_ = 0; 

    } catch (const tf2::TransformException &ex) {
        // TF lookup failed or data is frozen. Increment failure counter.
        tf_fail_count_++;
        
        if (tf_fail_count_ >= MAX_TF_FAILS) {
            hud_widget_->setTelemetryStatus(false);
            // Prevent integer overflow if disconnected for hours
            tf_fail_count_ = MAX_TF_FAILS; 
        }
    }
}

} // namespace insertion_base_station
PLUGINLIB_EXPORT_CLASS(insertion_base_station::FlightPanel, rviz_common::Panel)