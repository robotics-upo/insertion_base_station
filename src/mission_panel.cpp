#include "insertion_base_station/mission_panel.hpp"
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <QFileDialog>
#include <fstream>
#include <sstream>
#include <GeographicLib/LocalCartesian.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <QHeaderView> 
#include <QMessageBox> //

namespace insertion_base_station {

MissionPanel::MissionPanel(QWidget* parent) : rviz_common::Panel(parent) {
    QVBoxLayout* main_layout = new QVBoxLayout;
    tabs_ = new QTabWidget;

    tabs_->addTab(createDroneControlWidget(1), "DRONE 1");
    tabs_->addTab(createDroneControlWidget(2), "DRONE 2");

    main_layout->addWidget(tabs_);
    setLayout(main_layout);
}

QWidget* MissionPanel::createDroneControlWidget(int drone_id) {
    QWidget* widget = new QWidget;
    QVBoxLayout* layout = new QVBoxLayout;

    // --- STATUS ---
    drone_ui_map_[drone_id].status_label = new QLabel("STATUS: DISCONNECTED");
    drone_ui_map_[drone_id].status_label->setAlignment(Qt::AlignCenter);
    drone_ui_map_[drone_id].status_label->setStyleSheet("background-color: #2C3E50; color: #E74C3C; font-weight: bold; border-radius: 4px; padding: 10px;");
    layout->addWidget(drone_ui_map_[drone_id].status_label);

    // --- WATCHDOG TIMER ---
    drone_ui_map_[drone_id].watchdog_timer = new QTimer(this);
    drone_ui_map_[drone_id].watchdog_timer->setSingleShot(true);
    connect(drone_ui_map_[drone_id].watchdog_timer, &QTimer::timeout, [this, drone_id]() { onStatusTimeout(drone_id); });

    // --- FILE UPLOAD ---
    QHBoxLayout* f_layout = new QHBoxLayout;
    drone_ui_map_[drone_id].file_display = new QLineEdit();
    drone_ui_map_[drone_id].file_display->setReadOnly(true);
    QPushButton* browse_btn = new QPushButton("...");
    browse_btn->setFixedWidth(30);
    f_layout->addWidget(new QLabel("Mission File:"));
    f_layout->addWidget(drone_ui_map_[drone_id].file_display);
    f_layout->addWidget(browse_btn);
    layout->addLayout(f_layout);

    // --- CONTROL BUTTONS ---
    QGridLayout* grid = new QGridLayout;
    QPushButton* up_btn = new QPushButton("LOAD CSV");
    QPushButton* ex_btn = new QPushButton("EXECUTE MISSION");
    QPushButton* clear_btn = new QPushButton("CLEAR");
    QPushButton* abort_btn = new QPushButton("ABORT MISSION");
    QPushButton* rth_btn = new QPushButton("RTH (RETURN TO HOME)");

    ex_btn->setStyleSheet("background-color: #2E7D32; color: white; font-weight: bold;");
    rth_btn->setStyleSheet("background-color: #C62828; color: white; font-weight: bold;");
    clear_btn->setStyleSheet("background-color: #E67E22; color: white; font-weight: bold;");
    abort_btn->setStyleSheet("background-color: #C0392B; color: white; font-weight: bold;");

    grid->addWidget(up_btn, 0, 0);
    grid->addWidget(ex_btn, 0, 1);
    grid->addWidget(rth_btn, 1, 1);
    layout->addLayout(grid);

    // --- ABORT & CLEAR BUTTONS ---
    QHBoxLayout* danger_layout = new QHBoxLayout;
    danger_layout->addWidget(clear_btn);
    danger_layout->addWidget(abort_btn);
    grid->addLayout(danger_layout, 1, 0);

    // --- WAYPOINT TABLE ---
    drone_ui_map_[drone_id].wp_table = new QTableWidget(0, 4);
    drone_ui_map_[drone_id].wp_table->setHorizontalHeaderLabels(QStringList() << "Action" << "Latitude" << "Longitude" << "Alt (m)");
    drone_ui_map_[drone_id].wp_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    
    QHBoxLayout* table_btns_layout = new QHBoxLayout;
    QPushButton* add_wp_btn = new QPushButton("+ Add WP");
    QPushButton* rm_wp_btn = new QPushButton("- Remove WP");
    QPushButton* up_wp_btn = new QPushButton("↑ Up");
    QPushButton* down_wp_btn = new QPushButton("↓ Down");
    QPushButton* save_wp_btn = new QPushButton("SAVE TO CSV");
    save_wp_btn->setStyleSheet("background-color: #1976D2; color: white; font-weight: bold;");
   
    table_btns_layout->addWidget(add_wp_btn);
    table_btns_layout->addWidget(rm_wp_btn);
    table_btns_layout->addWidget(up_wp_btn);
    table_btns_layout->addWidget(down_wp_btn);
    table_btns_layout->addWidget(save_wp_btn);

    layout->addWidget(new QLabel("Waypoint List (Double-click to edit):"));
    layout->addWidget(drone_ui_map_[drone_id].wp_table);
    layout->addLayout(table_btns_layout);

    // --- ERROR MESSAGE LABEL ---
    drone_ui_map_[drone_id].error_label = new QLabel("");
    drone_ui_map_[drone_id].error_label->setStyleSheet("color: #D32F2F; font-weight: bold; background-color: #FFCDD2; padding: 5px; border-radius: 3px;");
    drone_ui_map_[drone_id].error_label->setAlignment(Qt::AlignCenter);
    drone_ui_map_[drone_id].error_label->hide();
    layout->addWidget(drone_ui_map_[drone_id].error_label);

    widget->setLayout(layout);

    // --- CONNECTIONS ---
    connect(browse_btn, &QPushButton::clicked, this, &MissionPanel::onUploadClicked);
    connect(up_btn, &QPushButton::clicked, this, &MissionPanel::onUploadClicked);
    connect(ex_btn, &QPushButton::clicked, this, &MissionPanel::onExecuteClicked);
    connect(clear_btn, &QPushButton::clicked, this, &MissionPanel::onClearClicked);
    connect(abort_btn, &QPushButton::clicked, this, &MissionPanel::onAbortClicked);
    connect(rth_btn, &QPushButton::clicked, this, &MissionPanel::onRTHClicked);
    connect(add_wp_btn, &QPushButton::clicked, this, &MissionPanel::onAddWpClicked);
    connect(rm_wp_btn, &QPushButton::clicked, this, &MissionPanel::onRemoveWpClicked);
    connect(up_wp_btn, &QPushButton::clicked, this, &MissionPanel::onMoveWpUpClicked);
    connect(down_wp_btn, &QPushButton::clicked, this, &MissionPanel::onMoveWpDownClicked);
    connect(save_wp_btn, &QPushButton::clicked, this, &MissionPanel::onSaveWpClicked);

connect(drone_ui_map_[drone_id].wp_table, &QTableWidget::itemChanged, [this, drone_id](QTableWidgetItem*) {drawMission(drone_id);});
    return widget;
}

void MissionPanel::onInitialize() {
    auto node = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

    interactive_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("mission_interactive_markers", node);

    node->declare_parameter("origin_lat", 39.794258);
    node->declare_parameter("origin_lon", -4.081346);
    node->declare_parameter("origin_alt", 536.3);

    node->declare_parameter("status_timeout_ms", 3000);
    node->declare_parameter("default_flight_alt", 20.0);
    node->declare_parameter("min_safe_alt", 5.0);

    origin_lat_ = node->get_parameter("origin_lat").as_double();
    origin_lon_ = node->get_parameter("origin_lon").as_double();
    origin_alt_ = node->get_parameter("origin_alt").as_double();

    status_timeout_ms_ = node->get_parameter("status_timeout_ms").as_int();
    default_flight_alt_ = node->get_parameter("default_flight_alt").as_double();
    min_safe_alt_ = node->get_parameter("min_safe_alt").as_double();

    RCLCPP_INFO(node->get_logger(), "MissionPanel Origin Set: %f, %f", origin_lat_, origin_lon_);
    
    marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("mission_visual_marker", 100);
    marker_array_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("mission_visual_marker_array", 10);

    mission_pub_d1_ = node->create_publisher<std_msgs::msg::String>("/drone1/execute_mission_file", 10);
    mission_pub_d2_ = node->create_publisher<std_msgs::msg::String>("/drone2/execute_mission_file", 10);

    status_sub_d1_ = node->create_subscription<std_msgs::msg::UInt8>(
        "/drone1/dji_sdk/display_mode", rclcpp::SensorDataQoS(), 
        [this](const std_msgs::msg::UInt8::SharedPtr msg){ statusCallback(msg, 1); });
    
    status_sub_d2_ = node->create_subscription<std_msgs::msg::UInt8>(
        "/drone2/dji_sdk/display_mode", rclcpp::SensorDataQoS(), 
        [this](const std_msgs::msg::UInt8::SharedPtr msg){ statusCallback(msg, 2); });

    rth_client_d1_ = node->create_client<std_srvs::srv::Trigger>("/drone1/dji_sdk/rth_task");
    rth_client_d2_ = node->create_client<std_srvs::srv::Trigger>("/drone2/dji_sdk/rth_task");

    clicked_point_sub_ = node->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 10, [this](const geometry_msgs::msg::PointStamped::SharedPtr msg){ clickedPointCallback(msg); });
    connect(this, &MissionPanel::pointReceived, this, &MissionPanel::addPointFromClick, Qt::QueuedConnection);

    // Start watchdogs in disconnected state
    onStatusTimeout(1);
    onStatusTimeout(2);
}

// ==========================================
// TELEMETRY STATUS & WATCHDOG
// ==========================================
void MissionPanel::statusCallback(const std_msgs::msg::UInt8::SharedPtr msg, int drone_id) {
    if(!drone_ui_map_.count(drone_id)) return;

    drone_ui_map_[drone_id].watchdog_timer->start(status_timeout_ms_);

    QString text; QString color;
    switch(msg->data) {
        case 0:  text = "STANDBY"; color = "#757575"; break;
        case 6:  text = "P-GPS (MANUAL)"; color = "#1976D2"; break;
        case 14: text = "AUTO: WAYPOINT MISSION"; color = "#388E3C"; break;
        case 17: text = "AUTO: RTH ACTIVE"; color = "#D32F2F"; break;
        default: text = "MODE " + QString::number(msg->data); color = "#F57C00"; break;
    }
    
    drone_ui_map_[drone_id].status_label->setText("STATUS: " + text);
    drone_ui_map_[drone_id].status_label->setStyleSheet(
        "background-color: " + color + "; color: white; font-weight: bold; border-radius: 4px; padding: 10px;");
}

void MissionPanel::onStatusTimeout(int drone_id) {
    if(drone_ui_map_.count(drone_id)) {
        drone_ui_map_[drone_id].status_label->setText("STATUS: DISCONNECTED");
        drone_ui_map_[drone_id].status_label->setStyleSheet(
            "background-color: #2C3E50; color: #E74C3C; font-weight: bold; border-radius: 4px; padding: 10px;");
    }
}

// ==========================================
// TABLE OPERATIONS
// ==========================================
void MissionPanel::onAddWpClicked() {
    int drone_id = tabs_->currentIndex() + 1;
    QTableWidget* table = drone_ui_map_[drone_id].wp_table;
    
    int row = table->rowCount();
    table->insertRow(row);
    
    // Create the dropdown menu for the Action
    QComboBox* action_combo = new QComboBox();
    action_combo->addItems({"WAYPOINT", "TAKEOFF", "LAND"});
    
    if (row == 0) action_combo->setCurrentText("TAKEOFF");
    else action_combo->setCurrentText("WAYPOINT");
    
    table->setCellWidget(row, 0, action_combo);
    
    // Fill coordinates
    table->blockSignals(true);
    table->setItem(row, 1, new QTableWidgetItem(QString::number(origin_lat_, 'f', 6)));
    table->setItem(row, 2, new QTableWidgetItem(QString::number(origin_lon_, 'f', 6)));
    table->setItem(row, 3, new QTableWidgetItem(QString::number(default_flight_alt_,'f', 1)));
    table->blockSignals(false);

    // Automatically redraw
    drawMission(drone_id);
}

void MissionPanel::onRemoveWpClicked() {
    int drone_id = tabs_->currentIndex() + 1;
    QTableWidget* table = drone_ui_map_[drone_id].wp_table;
    
    int current_row = table->currentRow();
    if (current_row >= 0) {
        table->removeRow(current_row);
    } else if (table->rowCount() > 0) {
        table->removeRow(table->rowCount() - 1);
    }

    drawMission(drone_id); 
}

// ==========================================
// MISSION LOGIC
// ==========================================
void MissionPanel::onUploadClicked() {
    int drone_id = tabs_->currentIndex() + 1;
    QString filename = QFileDialog::getOpenFileName(this, "Open Mission CSV", QDir::homePath(), "CSV (*.csv)");
    if (filename.isEmpty()) return;
    
    drone_ui_map_[drone_id].file_display->setText(filename);
    QTableWidget* table = drone_ui_map_[drone_id].wp_table;

    table->blockSignals(true);
    table->setRowCount(0);

    std::ifstream file(filename.toStdString());
    std::string line;
    int count = 0;
    while (std::getline(file, line)) {
        if (line.empty()) continue; 
        
        std::stringstream ss(line);
        std::string val; 
        double lat, lon, alt;
        try {
            std::string action_str;
            std::getline(ss, action_str, ',');
            std::getline(ss, val, ','); lat = std::stod(val);
            std::getline(ss, val, ','); lon = std::stod(val);
            std::getline(ss, val, ','); alt = std::stod(val);
            
            int row = table->rowCount();
            table->insertRow(row);
            
            QComboBox* action_combo = new QComboBox();
            action_combo->addItems({"WAYPOINT", "TAKEOFF", "LAND"});
            action_combo->setCurrentText(QString::fromStdString(action_str));
            table->setCellWidget(row, 0, action_combo);

            table->setItem(row, 1, new QTableWidgetItem(QString::number(lat, 'f', 8)));
            table->setItem(row, 2, new QTableWidgetItem(QString::number(lon, 'f', 8)));
            table->setItem(row, 3, new QTableWidgetItem(QString::number(alt, 'f', 2)));
            count++;
        } catch (...) { 
            RCLCPP_WARN(rclcpp::get_logger("mission"), "Drone %d: Ignoring malformed line in CSV.", drone_id);
            continue; 
        }
    }
    table->blockSignals(false);
    drawMission(drone_id);
}

void MissionPanel::drawMission(int drone_id) {
    if (drone_ui_map_.find(drone_id) == drone_ui_map_.end()) return;
    
    QTableWidget* table = drone_ui_map_[drone_id].wp_table;
    std::string mission_ns = "mission_drone_" + std::to_string(drone_id);

    // 1. Clear previous RViz markers
    visualization_msgs::msg::MarkerArray clear_array;

    visualization_msgs::msg::Marker clear_path;
    clear_path.header.frame_id = "map";
    clear_path.ns = mission_ns;
    clear_path.id = 9999;
    clear_path.action = visualization_msgs::msg::Marker::DELETE;
    clear_array.markers.push_back(clear_path);

    for (int i = 0; i < 100; ++i) {
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header.frame_id = "map";
        clear_marker.ns = mission_ns;
        clear_marker.action = visualization_msgs::msg::Marker::DELETE;

        clear_marker.id = i;
        clear_array.markers.push_back(clear_marker);

        clear_marker.id = i + 1000;
        clear_array.markers.push_back(clear_marker);

        clear_marker.id = i + 2000;
        clear_array.markers.push_back(clear_marker);
    }
    marker_array_pub_->publish(clear_array);

    for (int i = 0; i < 100; ++i) { 
        interactive_server_->erase("wp_" + std::to_string(drone_id) + "_" + std::to_string(i));
    }

    if (table->rowCount() == 0) {
        interactive_server_->applyChanges();
        return;
    }

    std::vector<MissionPoint> current_mission;
    for (int i = 0; i < table->rowCount(); ++i) {
        if (!table->item(i, 1) || !table->item(i, 2) || !table->item(i, 3)) return;

        bool ok_lat, ok_lon, ok_alt;
        MissionPoint p;
        p.latitude = table->item(i, 1)->text().toDouble(&ok_lat);
        p.longitude = table->item(i, 2)->text().toDouble(&ok_lon);
        p.altitude = table->item(i, 3)->text().toDouble(&ok_alt);

        if (!ok_lat || !ok_lon || !ok_alt) return;
        current_mission.push_back(p);
    }

    GeographicLib::LocalCartesian geo_converter(origin_lat_, origin_lon_, origin_alt_);
    auto now = rclcpp::Clock().now();
    float r = (drone_id == 1) ? 0.0 : 1.0;
    float g = 1.0;
    float b = (drone_id == 1) ? 1.0 : 0.0;

    visualization_msgs::msg::MarkerArray mission_array;

    // 2. Draw Path Line
    visualization_msgs::msg::Marker path_line;
    path_line.header.frame_id = "map";
    path_line.header.stamp = now;
    path_line.ns = mission_ns;
    path_line.id = 9999; 
    path_line.type = visualization_msgs::msg::Marker::LINE_LIST;
    path_line.action = visualization_msgs::msg::Marker::ADD;
    path_line.scale.x = 0.08; 
    path_line.color.r = r; path_line.color.g = g; path_line.color.b = b; path_line.color.a = 0.8;

    int wp_index = 0;
    geometry_msgs::msg::Point prev_p;
    bool has_prev = false;

    for (const auto& p : current_mission) {
        double x, y, z;
        double absolute_alt = origin_alt_ + p.altitude; 
        geo_converter.Forward(p.latitude, p.longitude, absolute_alt, x, y, z);
        
        geometry_msgs::msg::Point ros_p;
        ros_p.x = x; ros_p.y = y; ros_p.z = z;

        if (has_prev) {
            path_line.points.push_back(prev_p); 
            path_line.points.push_back(ros_p);
        }
        prev_p = ros_p;
        has_prev = true;

        makeInteractiveMarker(drone_id, wp_index, x, y, z);

        visualization_msgs::msg::Marker wp_sphere;
        wp_sphere.header.frame_id = "map";
        wp_sphere.header.stamp = now;
        wp_sphere.ns = mission_ns;
        wp_sphere.id = wp_index; 
        wp_sphere.type = visualization_msgs::msg::Marker::SPHERE;
        wp_sphere.action = visualization_msgs::msg::Marker::ADD;
        wp_sphere.scale.x = 0.6; wp_sphere.scale.y = 0.6; wp_sphere.scale.z = 0.6; 
        wp_sphere.color.r = r; wp_sphere.color.g = g; wp_sphere.color.b = b; wp_sphere.color.a = 1.0;
        wp_sphere.pose.position = ros_p;
        mission_array.markers.push_back(wp_sphere);

        visualization_msgs::msg::Marker wp_drop_line;
        wp_drop_line.header.frame_id = "map";
        wp_drop_line.header.stamp = now;
        wp_drop_line.ns = mission_ns;
        wp_drop_line.id = wp_index + 1000; 
        wp_drop_line.type = visualization_msgs::msg::Marker::LINE_LIST;
        wp_drop_line.action = visualization_msgs::msg::Marker::ADD;
        wp_drop_line.scale.x = 0.03; 
        wp_drop_line.color.r = r; wp_drop_line.color.g = g; wp_drop_line.color.b = b; wp_drop_line.color.a = 0.5;
        
        geometry_msgs::msg::Point p_ground;
        p_ground.x = x; p_ground.y = y; p_ground.z = 0.0; 
        wp_drop_line.points.push_back(p_ground);
        wp_drop_line.points.push_back(ros_p);
        mission_array.markers.push_back(wp_drop_line);

        visualization_msgs::msg::Marker wp_text;
        wp_text.header.frame_id = "map";
        wp_text.header.stamp = now;
        wp_text.ns = mission_ns;
        wp_text.id = wp_index + 2000; 
        wp_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        wp_text.action = visualization_msgs::msg::Marker::ADD;
        wp_text.text = "WP " + std::to_string(wp_index + 1);
        wp_text.scale.z = 0.5; 
        wp_text.color.r = 1.0; wp_text.color.g = 1.0; wp_text.color.b = 1.0; wp_text.color.a = 1.0;
        wp_text.pose.position.x = x; wp_text.pose.position.y = y; wp_text.pose.position.z = z + 1.0; 
        mission_array.markers.push_back(wp_text);

        wp_index++;
    }
    
    mission_array.markers.push_back(path_line);
    marker_array_pub_->publish(mission_array);

    interactive_server_->applyChanges();
}

void MissionPanel::onExecuteClicked() {
    int drone_id = tabs_->currentIndex() + 1;
    QTableWidget* table = drone_ui_map_[drone_id].wp_table;

    // 1. Pre-flight Validation (Table data)
    if (!validateMission(drone_id)) {
        RCLCPP_WARN(rclcpp::get_logger("mission"), "Drone %d: Pre-flight validation failed. Check red cells.", drone_id);
        return;
    }

    // 2. Telemetry / Datalink Check
    QString status_text = drone_ui_map_[drone_id].status_label->text();
    if (status_text.contains("DISCONNECTED")) {
        QMessageBox::critical(this, "Datalink Error", 
            QString("Cannot execute mission.\nDrone %1 is currently DISCONNECTED.\nPlease verify telemetry link.").arg(drone_id));
        RCLCPP_ERROR(rclcpp::get_logger("mission"), "Drone %d: Execution blocked (Disconnected).", drone_id);
        return;
    }

    // 3. Operator Confirmation (Two-step verification)
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "Confirm Execution", 
                                  QString("Are you sure you want to upload and EXECUTE the mission for Drone %1?").arg(drone_id),
                                  QMessageBox::Yes | QMessageBox::No);
    if (reply == QMessageBox::No) {
        RCLCPP_INFO(rclcpp::get_logger("mission"), "Drone %d: Mission execution aborted by operator.", drone_id);
        return; // Stop here if user cancels
    }

    RCLCPP_INFO(rclcpp::get_logger("mission"), "Drone %d: Processing mission with %d waypoints for execution...", drone_id, table->rowCount());

    // ==========================================================
    // DRONE COMMUNICATION LOGIC (SCP + SSH Execution)
    // ==========================================================

    // 1. Get mission file path and filename from the UI display
    QString local_full_path = drone_ui_map_[drone_id].file_display->text();
    if (local_full_path.isEmpty()) {
        QMessageBox::warning(this, "File Error", "No mission file selected or loaded in the table.");
        return;
    }

    QFileInfo file_info(local_full_path);
    QString filename = file_info.fileName();

    // 2. Define target drone IP and remote credentials
    // Drone 1 -> 192.168.25.5 | Drone 2 -> 192.168.25.7
    std::string ip = (drone_id == 1) ? "192.168.25.5" : "192.168.25.7";
    std::string user = "m600";
    std::string pass = "upo";
    std::string remote_dir = "/home/m600/csv/";
    std::string remote_file_path = remote_dir + filename.toStdString();

    // 3. Securely upload the mission file using sshpass (SCP)
    // Command: sshpass -p 'upo' scp /local/path/file.csv m600@IP:/home/m600/csv/
    std::string scp_cmd = "sshpass -p '" + pass + "' scp " + local_full_path.toStdString() + 
                          " " + user + "@" + ip + ":" + remote_dir;
    
    RCLCPP_INFO(rclcpp::get_logger("mission"), "Drone %d: Transferring mission file '%s' to %s", drone_id, filename.toStdString().c_str(), ip.c_str());
                          
    int scp_status = std::system(scp_cmd.c_str());
    if (scp_status != 0) {
        QMessageBox::critical(this, "Upload Failed", "Could not transfer the mission file to the drone. Check network connection.");
        RCLCPP_ERROR(rclcpp::get_logger("mission"), "Drone %d: SCP upload failed with exit code %d.", drone_id, scp_status);
        return;
    }

    // 4. Trigger remote mission execution via SSH (rostopic pub)
    // We use the -1 flag to publish once and exit immediately
    std::string ros_msg = "'{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: \"\"}, "
                          "goal_id: {stamp: {secs: 0, nsecs: 0}, id: \"\"}, "
                          "goal: {filename: {data: \"" + remote_file_path + "\"}}}'";

    std::string ssh_cmd = "sshpass -p '" + pass + "' ssh " + user + "@" + ip + 
                          " \"rostopic pub -1 /GPSNavigation/goal upo_actions/GPSNavigationActionGoal " + ros_msg + "\" &";

    RCLCPP_INFO(rclcpp::get_logger("mission"), "Drone %d: Triggering remote execution via SSH", drone_id);
    
    // Execute command in background (&) to prevent RViz UI from freezing
    std::system(ssh_cmd.c_str());

    // 5. Final operator feedback
    QMessageBox::information(this, "Mission Active", 
        QString("Mission '%1' successfully uploaded and triggered on Drone %2.").arg(filename).arg(drone_id));
}

void MissionPanel::onClearClicked() {
    int drone_id = tabs_->currentIndex() + 1;

    // Operator Confirmation for Clear
    QMessageBox::StandardButton reply = QMessageBox::question(this, "Clear UI", 
        "Clear the waypoint table and remove the route from the map?",
        QMessageBox::Yes | QMessageBox::No);
    if (reply == QMessageBox::No) return;

    QTableWidget* table = drone_ui_map_[drone_id].wp_table;
    table->setRowCount(0); 
    drone_ui_map_[drone_id].file_display->clear();
    drone_ui_map_[drone_id].error_label->hide();

    std::string mission_ns = "mission_drone_" + std::to_string(drone_id);

    // Clear mission using the MarkerArray
    visualization_msgs::msg::MarkerArray clear_array;
    visualization_msgs::msg::Marker clear_path;
    clear_path.header.frame_id = "map";
    clear_path.ns = mission_ns;
    clear_path.id = 9999;
    clear_path.action = visualization_msgs::msg::Marker::DELETE;
    clear_array.markers.push_back(clear_path);

    for (int i = 0; i < 100; ++i) {
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header.frame_id = "map";
        clear_marker.ns = mission_ns;
        clear_marker.action = visualization_msgs::msg::Marker::DELETE;

        clear_marker.id = i;         clear_array.markers.push_back(clear_marker);
        clear_marker.id = i + 1000;  clear_array.markers.push_back(clear_marker);
        clear_marker.id = i + 2000;  clear_array.markers.push_back(clear_marker);
    }
    marker_array_pub_->publish(clear_array);

    visualization_msgs::msg::Marker clear_rth;
    clear_rth.header.frame_id = "map";
    clear_rth.ns = "rth_target_" + std::to_string(drone_id);
    clear_rth.action = visualization_msgs::msg::Marker::DELETE;
    
    clear_rth.id = 0; marker_pub_->publish(clear_rth);
    clear_rth.id = 1; marker_pub_->publish(clear_rth);

    for (int i = 0; i < 100; ++i) {
        interactive_server_->erase("wp_" + std::to_string(drone_id) + "_" + std::to_string(i));
    }
    interactive_server_->applyChanges();

    RCLCPP_WARN(rclcpp::get_logger("mission"), "Drone %d: UI & RViz cleared by operator.", drone_id);
}

void MissionPanel::onAbortClicked() {
    int drone_id = tabs_->currentIndex() + 1;

    QString status_text = drone_ui_map_[drone_id].status_label->text();
    if (status_text.contains("DISCONNECTED")) {
        QMessageBox::critical(this, "Datalink Error", 
            QString("Cannot abort.\nDrone %1 is currently DISCONNECTED.").arg(drone_id));
        return;
    }

    // Operator Confirmation for Abort
    QMessageBox::StandardButton reply = QMessageBox::critical(this, "ABORT MISSION", 
        QString("Are you sure you want to ABORT the current mission for Drone %1?\nThe drone will stop and hover in place.").arg(drone_id),
        QMessageBox::Yes | QMessageBox::Cancel);
    if (reply == QMessageBox::Cancel) return;

    std::string ip = (drone_id == 1) ? "192.168.25.5" : "192.168.25.7";
    std::string user = "m600";
    std::string pass = "upo";
    
    // Abort command: Publish an empty GoalID to the cancel topic to stop the current mission
    std::string cancel_cmd = "rostopic pub -1 /GPSNavigation/cancel actionlib_msgs/GoalID \\\"{}\\\"";
    std::string ssh_cmd = "sshpass -p '" + pass + "' ssh " + user + "@" + ip + " \"" + cancel_cmd + "\" &";

    RCLCPP_WARN(rclcpp::get_logger("mission"), "Drone %d: MISSION ABORTED BY OPERATOR", drone_id);
    std::system(ssh_cmd.c_str());

    QMessageBox::information(this, "Mission Aborted", 
        QString("Abort command sent to Drone %1. It will cancel the trajectory and hover.").arg(drone_id));
}

void MissionPanel::onRTHClicked() {
    int drone_id = tabs_->currentIndex() + 1;

    // 1. Telemetry / Datalink Check
    QString status_text = drone_ui_map_[drone_id].status_label->text();
    if (status_text.contains("DISCONNECTED")) {
        QMessageBox::critical(this, "Datalink Error", 
            QString("Cannot trigger RTH.\nDrone %1 is currently DISCONNECTED.").arg(drone_id));
        return;
    }

    // 2. Critical Operator Confirmation
    QMessageBox::StandardButton reply;
    reply = QMessageBox::critical(this, "CRITICAL ACTION: RTH", 
                                  QString("Command Drone %1 to Return To Home (RTH) immediately?").arg(drone_id),
                                  QMessageBox::Yes | QMessageBox::Cancel);
    if (reply == QMessageBox::Cancel) {
        return;
    }
    
    // 3. Automatically clear the current path, table, and mission visualizations
    QTableWidget* table = drone_ui_map_[drone_id].wp_table;
    table->setRowCount(0); 
    drone_ui_map_[drone_id].file_display->clear();
    drone_ui_map_[drone_id].error_label->hide();

    std::string mission_ns = "mission_drone_" + std::to_string(drone_id);

    visualization_msgs::msg::MarkerArray clear_mission_array;
    visualization_msgs::msg::Marker clear_path;
    clear_path.header.frame_id = "map";
    clear_path.ns = mission_ns;
    clear_path.id = 9999;
    clear_path.action = visualization_msgs::msg::Marker::DELETE;
    clear_mission_array.markers.push_back(clear_path);

    for (int i = 0; i < 100; ++i) {
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header.frame_id = "map";
        clear_marker.ns = mission_ns;
        clear_marker.action = visualization_msgs::msg::Marker::DELETE;

        clear_marker.id = i;         clear_mission_array.markers.push_back(clear_marker);
        clear_marker.id = i + 1000;  clear_mission_array.markers.push_back(clear_marker);
        clear_marker.id = i + 2000;  clear_mission_array.markers.push_back(clear_marker);
    }
    marker_array_pub_->publish(clear_mission_array);

    for (int i = 0; i < 100; ++i) {
        interactive_server_->erase("wp_" + std::to_string(drone_id) + "_" + std::to_string(i));
    }
    interactive_server_->applyChanges();

    // 4. Calculate local coordinates for origin
    GeographicLib::LocalCartesian geo_converter(origin_lat_, origin_lon_, origin_alt_);
    double x, y, z;
    geo_converter.Forward(origin_lat_, origin_lon_, origin_alt_, x, y, z);
    
    // 5. Clear previous RTH visualizations
    visualization_msgs::msg::Marker clear_msg;
    clear_msg.header.frame_id = "map";
    clear_msg.ns = "rth_target_" + std::to_string(drone_id);
    clear_msg.action = visualization_msgs::msg::Marker::DELETE;
    
    visualization_msgs::msg::MarkerArray clear_array;
    clear_array.markers.push_back(clear_msg);
    marker_array_pub_->publish(clear_array);

    visualization_msgs::msg::MarkerArray rth_array;
    std::string rth_ns = "rth_target_" + std::to_string(drone_id);

    // 6. Draw the Beacon
    visualization_msgs::msg::Marker rth_beacon;
    rth_beacon.header.frame_id = "map";
    rth_beacon.header.stamp = rclcpp::Clock().now();
    rth_beacon.ns = rth_ns;
    rth_beacon.id = 0;
    rth_beacon.type = visualization_msgs::msg::Marker::CYLINDER;
    rth_beacon.action = visualization_msgs::msg::Marker::ADD;    
    rth_beacon.scale.x = 0.5; rth_beacon.scale.y = 0.5; rth_beacon.scale.z = 100.0; 
    rth_beacon.color.r = 1.0; rth_beacon.color.g = 0.0; rth_beacon.color.b = 1.0; rth_beacon.color.a = 0.8; 
    rth_beacon.pose.position.x = x; 
    rth_beacon.pose.position.y = y; 
    rth_beacon.pose.position.z = 50.0;
    rth_array.markers.push_back(rth_beacon);

    // 7. Draw the Text Label 
    visualization_msgs::msg::Marker rth_text;
    rth_text.header.frame_id = "map";
    rth_text.header.stamp = rclcpp::Clock().now();
    rth_text.ns = rth_ns;
    rth_text.id = 1;
    rth_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    rth_text.action = visualization_msgs::msg::Marker::ADD;
    rth_text.text = "RTH";
    rth_text.scale.z = 3.0; 
    rth_text.color.r = 1.0; rth_text.color.g = 1.0; rth_text.color.b = 1.0; rth_text.color.a = 1.0;
    rth_text.pose.position.x = x; 
    rth_text.pose.position.y = y; 
    rth_text.pose.position.z = 105.0;
    rth_array.markers.push_back(rth_text);
    
    marker_array_pub_->publish(rth_array);

    // 8. DRONE RTH COMMUNICATION LOGIC (Cancel + Upload)

    double safe_rth_altitude = 30.0; // Safe altitude

    // Create temporary CSV file for RTH mission
    QString local_filename = QString("/tmp/emergency_rth_d%1.csv").arg(drone_id);
    std::ofstream rth_file(local_filename.toStdString());
    if (rth_file.is_open()) {
        rth_file << "LAND," << origin_lat_ << "," << origin_lon_ << "," << safe_rth_altitude << "\n";
        rth_file.close();
    } else {
        QMessageBox::critical(this, "RTH Error", "Could not create emergency CSV file.");
        return;
    }

    std::string ip = (drone_id == 1) ? "192.168.25.5" : "192.168.25.7";
    std::string user = "m600";
    std::string pass = "upo";
    std::string remote_file_path = "/home/m600/csv/emergency_rth.csv";

    // SCP Command
    std::string scp_cmd = "sshpass -p '" + pass + "' scp " + local_filename.toStdString() + 
                          " " + user + "@" + ip + ":" + remote_file_path;
    
    RCLCPP_INFO(rclcpp::get_logger("mission"), "Drone %d: Uploading Emergency RTH File...", drone_id);
    int scp_status = std::system(scp_cmd.c_str());
    if (scp_status != 0) {
        QMessageBox::critical(this, "RTH Error", "Failed to upload emergency RTH file to the drone.");
        return;
    }

    // SSH Commands
    // Cancel current mission
    std::string cancel_cmd = "rostopic pub -1 /GPSNavigation/cancel actionlib_msgs/GoalID \\\"{}\\\"";
    
    // Start new RTH mission
    std::string ros_msg = "'{header: {seq: 0, stamp: {secs: 0, nsecs: 0}, frame_id: \"\"}, "
                          "goal_id: {stamp: {secs: 0, nsecs: 0}, id: \"\"}, "
                          "goal: {filename: {data: \"" + remote_file_path + "\"}}}'";
    std::string start_cmd = "rostopic pub -1 /GPSNavigation/goal upo_actions/GPSNavigationActionGoal " + ros_msg;

    // Combine commands into a single SSH execution
    std::string ssh_cmd = "sshpass -p '" + pass + "' ssh " + user + "@" + ip + 
                          " \"" + cancel_cmd + " && sleep 1 && " + start_cmd + "\" &";

    RCLCPP_WARN(rclcpp::get_logger("mission"), "Drone %d: Executing EMERGENCY RTH Cancel & Start", drone_id);
    std::system(ssh_cmd.c_str());

    QMessageBox::information(this, "RTH Active", 
        QString("Emergency RTH triggered!\nDrone %1 is aborting current mission and returning to Base Coordinates.").arg(drone_id));
}

void MissionPanel::onSaveWpClicked() {
    int drone_id = tabs_->currentIndex() + 1;
    QTableWidget* table = drone_ui_map_[drone_id].wp_table;

    if (table->rowCount() == 0) return;

    QString filename = QFileDialog::getSaveFileName(this, "Save Mission CSV", QDir::homePath(), "CSV (*.csv)");
    if (filename.isEmpty()) return; 

    if (!filename.endsWith(".csv", Qt::CaseInsensitive)) filename += ".csv";

    std::ofstream file(filename.toStdString());
    if (!file.is_open()) return;

    for (int i = 0; i < table->rowCount(); ++i) {
        QComboBox* combo = qobject_cast<QComboBox*>(table->cellWidget(i, 0));
        QString action = combo ? combo->currentText() : "WAYPOINT";
        
        QString lat = table->item(i, 1) ? table->item(i, 1)->text() : "0.0";
        QString lon = table->item(i, 2) ? table->item(i, 2)->text() : "0.0";
        QString alt = table->item(i, 3) ? table->item(i, 3)->text() : "0.0";
        
        file << action.toStdString() << "," << lat.toStdString() << "," << lon.toStdString() << "," << alt.toStdString() << "\n";
    }

    file.close();
}

// ==========================================
// WAYPOINT REORDERING LOGIC
// ==========================================
void MissionPanel::onMoveWpUpClicked() {
    int drone_id = tabs_->currentIndex() + 1;
    QTableWidget* table = drone_ui_map_[drone_id].wp_table;
    
    int current_row = table->currentRow();
    if (current_row > 0) {
        swapRows(current_row, current_row - 1, drone_id);
        table->selectRow(current_row - 1); // Keep the row selected as it moves up
        drawMission(drone_id);
    }
}

void MissionPanel::onMoveWpDownClicked() {
    int drone_id = tabs_->currentIndex() + 1;
    QTableWidget* table = drone_ui_map_[drone_id].wp_table;
    
    int current_row = table->currentRow();
    if (current_row >= 0 && current_row < table->rowCount() - 1) {
        swapRows(current_row, current_row + 1, drone_id);
        table->selectRow(current_row + 1); // Keep the row selected as it moves down
        drawMission(drone_id);
    }
}

void MissionPanel::swapRows(int row1, int row2, int drone_id) {
    QTableWidget* table = drone_ui_map_[drone_id].wp_table;
    table->blockSignals(true);

    // 1. Swap the QComboBox (Action) values
    QComboBox* combo1 = qobject_cast<QComboBox*>(table->cellWidget(row1, 0));
    QComboBox* combo2 = qobject_cast<QComboBox*>(table->cellWidget(row2, 0));
    if (combo1 && combo2) {
        QString temp_action = combo1->currentText();
        combo1->setCurrentText(combo2->currentText());
        combo2->setCurrentText(temp_action);
    }

    // 2. Swap the text items (Lat, Lon, Alt)
    for (int col = 1; col < 4; ++col) {
        QTableWidgetItem* item1 = table->takeItem(row1, col);
        QTableWidgetItem* item2 = table->takeItem(row2, col);
        
        table->setItem(row1, col, item2);
        table->setItem(row2, col, item1);
    }
    table->blockSignals(false);
}

// ==========================================
// PRE-FLIGHT VALIDATION LOGIC
// ==========================================
bool MissionPanel::validateMission(int drone_id) {
    QTableWidget* table = drone_ui_map_[drone_id].wp_table;
    bool all_valid = true;
    QString coord_error_msg = "Invalid entries in row(s): ";
    QString logic_error_msg = "";
    bool has_coord_errors = false;
    
    int row_count = table->rowCount();

    for (int i = 0; i < row_count; ++i) {
        bool lat_ok, lon_ok, alt_ok;
        table->item(i, 1)->text().toDouble(&lat_ok);
        table->item(i, 2)->text().toDouble(&lon_ok);
        table->item(i, 3)->text().toDouble(&alt_ok);

        if (!lat_ok || !lon_ok || !alt_ok) {
            table->item(i, 1)->setBackground(QColor(255, 200, 200));
            table->item(i, 2)->setBackground(QColor(255, 200, 200));
            table->item(i, 3)->setBackground(QColor(255, 200, 200));
            all_valid = false;
            has_coord_errors = true;
            coord_error_msg += QString::number(i + 1) + " ";
        } else {
            table->item(i, 1)->setBackground(Qt::white);
            table->item(i, 2)->setBackground(Qt::white);
            table->item(i, 3)->setBackground(Qt::white);
        }

        QComboBox* combo = qobject_cast<QComboBox*>(table->cellWidget(i, 0));
        if (combo) {
            QString action = combo->currentText();
            
            if (action == "TAKEOFF" && i != 0) {
                all_valid = false;
                if (!logic_error_msg.contains("TAKEOFF")) {
                    logic_error_msg += "TAKEOFF must be the first point. ";
                }
            }
            
            if (action == "LAND" && i != (row_count - 1)) {
                all_valid = false;
                if (!logic_error_msg.contains("LAND")) {
                    logic_error_msg += "LAND must be the last point. ";
                }
            }
        }
    }

    if (!all_valid) {
        QString final_error_msg = "";
        if (has_coord_errors) final_error_msg += coord_error_msg + "\n";
        if (!logic_error_msg.isEmpty()) final_error_msg += logic_error_msg;
        
        drone_ui_map_[drone_id].error_label->setText(final_error_msg.trimmed());
        drone_ui_map_[drone_id].error_label->show();
    } else {
        drone_ui_map_[drone_id].error_label->hide();
    }

    return all_valid;
}

// ==========================================
// INTERACTIVE MARKERS (DRAG & DROP)
// ==========================================
void MissionPanel::makeInteractiveMarker(int drone_id, int wp_index, double x, double y, double z) {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    int_marker.header.stamp = rclcpp::Clock().now();
    int_marker.name = "wp_" + std::to_string(drone_id) + "_" + std::to_string(wp_index);
    
    int_marker.pose.position.x = x;
    int_marker.pose.position.y = y;
    int_marker.pose.position.z = z;

    visualization_msgs::msg::Marker visual_marker;
    visual_marker.type = visualization_msgs::msg::Marker::SPHERE;
    visual_marker.scale.x = 0.8; 
    visual_marker.scale.y = 0.8;
    visual_marker.scale.z = 0.8;
    
    visual_marker.color.r = (drone_id == 1) ? 0.0 : 1.0;
    visual_marker.color.g = 1.0;
    visual_marker.color.b = (drone_id == 1) ? 1.0 : 0.0;
    visual_marker.color.a = 1.0;

    visualization_msgs::msg::InteractiveMarkerControl move_control;
    move_control.always_visible = true; 
    move_control.name = "move_xy";
    move_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
    
    // XY plane
    move_control.orientation.w = 0.7071;
    move_control.orientation.x = 0.0;
    move_control.orientation.y = 0.7071;
    move_control.orientation.z = 0.0;
    
    move_control.markers.push_back(visual_marker);
    int_marker.controls.push_back(move_control);

    interactive_server_->insert(int_marker);
    interactive_server_->setCallback(int_marker.name, std::bind(&MissionPanel::processMarkerFeedback, this, std::placeholders::_1));
}

void MissionPanel::processMarkerFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr &feedback) {
    if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP) {
        
        int drone_id, wp_index;
        if (sscanf(feedback->marker_name.c_str(), "wp_%d_%d", &drone_id, &wp_index) == 2) {
            
            GeographicLib::LocalCartesian geo_converter(origin_lat_, origin_lon_, origin_alt_);
            double lat, lon, alt;
            geo_converter.Reverse(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z, lat, lon, alt);

            QTimer::singleShot(0, this, [this, drone_id, wp_index, lat, lon]() {
                QTableWidget* table = drone_ui_map_[drone_id].wp_table;
                if (wp_index < table->rowCount()) {
                    // Update the table with new coordinates
                    table->blockSignals(true);
                    table->item(wp_index, 1)->setText(QString::number(lat, 'f', 6));
                    table->item(wp_index, 2)->setText(QString::number(lon, 'f', 6));
                    table->blockSignals(false);

                    drawMission(drone_id);
                }
            });
        }
    }
}

// ==========================================
// CLICK MAP OPERATIONS
// ==========================================
void MissionPanel::clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    GeographicLib::LocalCartesian geo_converter(origin_lat_, origin_lon_, origin_alt_);
    double lat, lon, absolute_alt;
    
    geo_converter.Reverse(msg->point.x, msg->point.y, msg->point.z, lat, lon, absolute_alt);
    double rel_alt = msg->point.z > min_safe_alt_ ? msg->point.z : default_flight_alt_;

    Q_EMIT pointReceived(lat, lon, rel_alt);
}

void MissionPanel::addPointFromClick(double lat, double lon, double alt) {
    int drone_id = tabs_->currentIndex() + 1;
    QTableWidget* table = drone_ui_map_[drone_id].wp_table;
    
    int row = table->rowCount();
    table->insertRow(row);
    
    QComboBox* action_combo = new QComboBox();
    action_combo->addItems({"WAYPOINT", "TAKEOFF", "LAND"});
    
    if (row == 0) action_combo->setCurrentText("TAKEOFF");
    else action_combo->setCurrentText("WAYPOINT");
    
    table->setCellWidget(row, 0, action_combo);

    table->blockSignals(true);
    table->setItem(row, 1, new QTableWidgetItem(QString::number(lat, 'f', 6)));
    table->setItem(row, 2, new QTableWidgetItem(QString::number(lon, 'f', 6)));
    table->setItem(row, 3, new QTableWidgetItem(QString::number(alt, 'f', 2)));
    table->blockSignals(false);

    // Automatically redraw
    drawMission(drone_id);
}

} // namespace insertion_base_station
PLUGINLIB_EXPORT_CLASS(insertion_base_station::MissionPanel, rviz_common::Panel)