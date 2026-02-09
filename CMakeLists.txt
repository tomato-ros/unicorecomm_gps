cmake_minimum_required(VERSION 3.10)

project(unicorecomm_gps)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# 查找依赖包
find_package(ament_cmake_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rtcm_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)

# 生成静态库
add_library(unicorecomm_gps
        src/UnicorecommGpsNode.cpp
        include/unicorecomm_gps/UnicorecommGpsNode.hpp)

# 包含目标头文件
target_include_directories(unicorecomm_gps PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

# 链接目标静态库
target_link_libraries(unicorecomm_gps PUBLIC
  ${geometry_msgs_TARGETS}
  ${rcl_interfaces_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  ${nmea_msgs_TARGETS}
  ${rtcm_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  tf2::tf2
)

# 安装目标头文件
install(TARGETS unicorecomm_gps EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 生成可执行程序
add_executable(unicorecomm_gps_node src/UnicorecommGpsNodeMain.cpp
        src/UnicorecommGpsNode.cpp
        include/unicorecomm_gps/UnicorecommGpsNode.hpp)

# 链接可执行程序
target_link_libraries(unicorecomm_gps_node PRIVATE
  rclcpp::rclcpp
  unicorecomm_gps
)

rclcpp_components_register_nodes(unicorecomm_gps "unicorecomm_gps_node::UnicorecommGpsNode")

# 安装可执行程序
install(TARGETS
  unicorecomm_gps_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件目录
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# 安装启动脚本和配置文件
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories("include/${PROJECT_NAME}")

ament_export_targets(export_${PROJECT_NAME})

ament_package()
