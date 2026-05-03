#include "insertion_base_station/battery_panel.hpp"
#include "ui_battery_panel.h" 

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <algorithm> 
#include <algorithm>

namespace insertion_base_station {

BatteryPanel::BatteryPanel(QWidget * parent)
  : rviz_common::Panel(parent), ui_(new Ui::Form) 
{
  ui_->setupUi(this);

  // Initialize watchdog timers
  watchdog_timer1_ = new QTimer(this);
  watchdog_timer1_->setSingleShot(true); // Only fire once if time runs out
  
  watchdog_timer2_ = new QTimer(this);
  watchdog_timer2_->setSingleShot(true);

  // Connect timeouts to their respective slots
  connect(watchdog_timer1_, &QTimer::timeout, this, &BatteryPanel::onBattery1Timeout);
  connect(watchdog_timer2_, &QTimer::timeout, this, &BatteryPanel::onBattery2Timeout);
}

BatteryPanel::~BatteryPanel() {
  delete ui_;
}

void BatteryPanel::onInitialize() {
  auto node_ptr = getDisplayContext()->getRosNodeAbstraction().lock();
  if (node_ptr) {
    auto node = node_ptr->get_raw_node();
    
    node->declare_parameter("battery_timeout_ms", 5000);
    battery_timeout_ms_ = node->get_parameter("battery_timeout_ms").as_int();
    
    RCLCPP_INFO(node->get_logger(), "Battery watchdog set to %d ms", battery_timeout_ms_);
    
    // Subscribe to drone 1 battery topic
    battery_sub1_ = node->create_subscription<sensor_msgs::msg::BatteryState>(
      "/drone1/dji_sdk/battery_state", rclcpp::SensorDataQoS(),
      std::bind(&BatteryPanel::battery1Callback, this, std::placeholders::_1));

    // Subscribe to drone 2 battery topic
    battery_sub2_ = node->create_subscription<sensor_msgs::msg::BatteryState>(
      "/drone2/dji_sdk/battery_state", rclcpp::SensorDataQoS(),
      std::bind(&BatteryPanel::battery2Callback, this, std::placeholders::_1));
  }
  
  // Connect ROS callbacks (Signals) to UI updates (Slots) safely across threads
  connect(this, &BatteryPanel::battery1Updated, this, &BatteryPanel::updateBattery1UI);
  connect(this, &BatteryPanel::battery2Updated, this, &BatteryPanel::updateBattery2UI);

  // Set initial "NO DATA" state until first message arrives
  onBattery1Timeout();
  onBattery2Timeout();
}

// ==========================================
// ROS CALLBACKS (Triggered when data arrives)
// ==========================================
void BatteryPanel::battery1Callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    double pct = msg->percentage;
    // Standard ROS 2 battery percentage is between 0.0 and 1.0. 
    // If the value is within this range, we convert it to 0-100.
    if (pct >= 0.0 && pct <= 1.0) {
        pct *= 100.0;
    }
    Q_EMIT battery1Updated(pct);
}

void BatteryPanel::battery2Callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    double pct = msg->percentage;
    if (pct >= 0.0 && pct <= 1.0) {
        pct *= 100.0;
    }
    Q_EMIT battery2Updated(pct);
}

// ==========================================
// UI UPDATES & TIMER RESETS
// ==========================================
void BatteryPanel::updateBattery1UI(double percentage) {
// 1. Reset the watchdog timer
    watchdog_timer1_->start(battery_timeout_ms_); 
    
    // 2. Restore normal text format (e.g. "85%")
    ui_->battery_bar1->setFormat("%p%"); 
    
    // 3. Update value and color
    int val = std::clamp(static_cast<int>(percentage), 0, 100);
    ui_->battery_bar1->setValue(val);
    ui_->battery_bar1->setStyleSheet(getStyleByValue(val));
}

void BatteryPanel::updateBattery2UI(double percentage) {
watchdog_timer2_->start(battery_timeout_ms_); 
    ui_->battery_bar2->setFormat("%p%"); 
    
    int val = std::clamp(static_cast<int>(percentage), 0, 100);
    ui_->battery_bar2->setValue(val);
    ui_->battery_bar2->setStyleSheet(getStyleByValue(val));
}

// ==========================================
// WATCHDOG TIMEOUT SLOTS (Disconnection)
// ==========================================
void BatteryPanel::onBattery1Timeout() {
    auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    RCLCPP_WARN(node->get_logger(), "Drone 1: BATTERY TELEMETRY LOST! (No data for 5s)");

    ui_->battery_bar1->setValue(0);
    ui_->battery_bar1->setFormat("NO DATA"); // Overrides the percentage text
    ui_->battery_bar1->setStyleSheet(
        "QProgressBar { text-align: center; font-weight: bold; border: 1px solid grey; border-radius: 3px; color: #E74C3C; background-color: #2C3E50; }"
        "QProgressBar::chunk { background-color: #7F8C8D; }"
    );
}

void BatteryPanel::onBattery2Timeout() {
    auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    RCLCPP_WARN(node->get_logger(), "Drone 2: BATTERY TELEMETRY LOST! (No data for 5s)");

    ui_->battery_bar2->setValue(0);
    ui_->battery_bar2->setFormat("NO DATA");
    ui_->battery_bar2->setStyleSheet(
        "QProgressBar { text-align: center; font-weight: bold; border: 1px solid grey; border-radius: 3px; color: #E74C3C; background-color: #2C3E50; }"
        "QProgressBar::chunk { background-color: #7F8C8D; }"
    );
}

// ==========================================
// HELPER FUNCTIONS
// ==========================================
QString BatteryPanel::getStyleByValue(int val) {
    QString color_style;
    if (val > 50) {
        color_style = "QProgressBar::chunk { background-color: #2ECC71; }"; // Green
    } else if (val > 20) {
        color_style = "QProgressBar::chunk { background-color: #F1C40F; }"; // Yellow
    } else {
        color_style = "QProgressBar::chunk { background-color: #E74C3C; }"; // Red
    }
    
    return "QProgressBar { text-align: center; font-weight: bold; border: 1px solid grey; border-radius: 3px; }" + color_style;
}

} // namespace insertion_base_station

PLUGINLIB_EXPORT_CLASS(insertion_base_station::BatteryPanel, rviz_common::Panel)