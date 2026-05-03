cmake_minimum_required(VERSION 3.8)
project(insertion_base_station)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- FIND DEPENDENCIES ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)                 
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)

# --- QT INTEGRATION ---
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ==========================================
# 1. INDEPENDENT NODES
# ==========================================
add_executable(global_to_local src/global_to_local.cpp)
ament_target_dependencies(global_to_local 
  rclcpp tf2_ros sensor_msgs geographic_msgs nav_msgs
)
target_link_libraries(global_to_local Geographic)

# ==========================================
# 2. RVIZ PLUGINS (ALL UNIFIED PANELS)
# ==========================================
add_library(insertion_base_station_panels SHARED
  src/mission_panel.cpp
  include/insertion_base_station/mission_panel.hpp
  src/battery_panel.cpp
  include/insertion_base_station/battery_panel.hpp
  ui/battery_panel.ui
  src/flight_panel.cpp
  include/insertion_base_station/flight_panel.hpp
  src/layer_panel.cpp
  include/insertion_base_station/layer_panel.hpp
)

target_include_directories(insertion_base_station_panels PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# We group all the ROS dependencies used by the different panels
ament_target_dependencies(insertion_base_station_panels 
  rclcpp 
  rviz_common 
  rviz_rendering 
  sensor_msgs
  visualization_msgs 
  interactive_markers
  std_msgs 
  std_srvs 
  geometry_msgs
  tf2 
  tf2_ros 
  pluginlib
)

# We group all external libraries (Qt and GeographicLib)
target_link_libraries(insertion_base_station_panels 
  Qt5::Core 
  Qt5::Gui 
  Qt5::Widgets 
  Geographic
)

# ==========================================
# 3. EXPORTS E INSTALLS
# ==========================================
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

# Install C++ nodes
install(TARGETS
  global_to_local
  DESTINATION lib/${PROJECT_NAME}
)

# Install the unified panel library
install(TARGETS insertion_base_station_panels
  EXPORT export_insertion_base_station_panels
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install static directories
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY cfg DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY include/ DESTINATION include)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install python scripts
install(PROGRAMS
  scripts/radar_converter.py
  scripts/camera_fix.py
  scripts/tf_fixer.py
  scripts/gps_publisher.py
  scripts/camera_mux.py
  scripts/asset_labels.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()