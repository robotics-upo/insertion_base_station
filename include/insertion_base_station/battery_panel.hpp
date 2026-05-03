#ifndef BATTERY_PANEL_HPP
#define BATTERY_PANEL_HPP

#include <rviz_common/panel.hpp>
#include <QtWidgets>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

namespace Ui { class Form; } 

namespace insertion_base_station {

class BatteryPanel : public rviz_common::Panel {
  Q_OBJECT
public:
  explicit BatteryPanel(QWidget * parent = nullptr);
  virtual ~BatteryPanel();
  virtual void onInitialize() override;

Q_SIGNALS:
  // Signals to safely update the UI from the ROS thread
  void battery1Updated(double percentage);
  void battery2Updated(double percentage);

protected Q_SLOTS:
  // Slots connected to the signals to update the Progress Bars
  void updateBattery1UI(double percentage);
  void updateBattery2UI(double percentage);

  // Watchdog timeout handlers
  void onBattery1Timeout();
  void onBattery2Timeout();

protected:
  Ui::Form* ui_;
  int battery_timeout_ms_;

  // Watchdog timers
  QTimer* watchdog_timer1_;
  QTimer* watchdog_timer2_;
  
  // ROS 2 Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub1_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub2_;  
  
  // ROS 2 Callbacks
  void battery1Callback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void battery2Callback(const sensor_msgs::msg::BatteryState::SharedPtr msg);

  // Helper function for dynamic styling
  QString getStyleByValue(int val);
};

} // namespace insertion_base_station

#endif // BATTERY_PANEL_HPP