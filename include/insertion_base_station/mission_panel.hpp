#ifndef MISSION_PANEL_HPP
#define MISSION_PANEL_HPP

#include <rviz_common/panel.hpp>
#include <QtWidgets>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <map>
#include <vector>
#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

namespace insertion_base_station {

class MissionPanel : public rviz_common::Panel {
    Q_OBJECT
public:
    explicit MissionPanel(QWidget * parent = nullptr);
    virtual void onInitialize() override;

Q_SIGNALS:
    void pointReceived(double lat, double lon, double alt);

protected Q_SLOTS:
    void onUploadClicked();
    void onExecuteClicked();
    void onAbortClicked();
    void onClearClicked();
    void onRTHClicked();
    void onAddWpClicked();
    void onRemoveWpClicked();
    void onSaveWpClicked();
    void onMoveWpUpClicked();
    void onMoveWpDownClicked();
    void addPointFromClick(double lat, double lon, double alt);
    void drawMission(int drone_id);

private:
    void swapRows(int row1, int row2, int drone_id);
    bool validateMission(int drone_id);

    void makeInteractiveMarker(int drone_id, int wp_index, double x, double y, double z);
    void processMarkerFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback);

    struct DroneWidgets {
        QLabel* status_label;
        QLineEdit* file_display;
        QTableWidget* wp_table;
        QTimer* watchdog_timer; 
        QLabel* error_label;
    };

    struct MissionPoint { double latitude, longitude, altitude; };

    QTabWidget* tabs_;
    std::map<int, DroneWidgets> drone_ui_map_;

    // ROS 2
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> interactive_server_;
    
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_pub_d1_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_pub_d2_;

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr status_sub_d1_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr status_sub_d2_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
    
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr rth_client_d1_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr rth_client_d2_;

    void statusCallback(const std_msgs::msg::UInt8::SharedPtr msg, int drone_id);
    void onStatusTimeout(int drone_id);
    void clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    QWidget* createDroneControlWidget(int drone_id);
    
    // Parameters
    double origin_lat_;
    double origin_lon_;
    double origin_alt_;
    int status_timeout_ms_;
    double default_flight_alt_;
    double min_safe_alt_;
};

} // namespace insertion_base_station
#endif