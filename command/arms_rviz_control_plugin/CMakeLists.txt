cmake_minimum_required(VERSION 3.10)
project(arms_rviz_control_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_AUTOMOC OFF)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(arms_controller_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(arms_ros2_control_msgs REQUIRED)

# feat/foxy: Ubuntu 20.04 / Focal never ships Qt6 (rviz2 there is Qt5-only) and
# its apt cmake (3.16.3) predates cmake_language(CALL ...) (3.18+), which the
# upstream Qt5/Qt6 dynamic-dispatch build used. Hardcoding Qt5 avoids both
# problems; if this branch is ever built against a newer distro, redo the
# dynamic dispatch from ocs2_ros2's pattern instead of relaxing this back.
find_package(Qt5 REQUIRED COMPONENTS Widgets)

qt5_wrap_cpp(MOC_FILES
        include/arms_rviz_control_plugin/ocs2_fsm_panel.hpp
        include/arms_rviz_control_plugin/gripper_control_panel.hpp
        include/arms_rviz_control_plugin/joint_control_panel.hpp
        include/arms_rviz_control_plugin/wbc_capability_panel.hpp
        include/arms_rviz_control_plugin/switch_button.hpp
)

# 设置源代码文件列表
set(SRC_FILES
        src/ocs2_fsm_panel.cpp
        src/gripper_control_panel.cpp
        src/joint_control_panel.cpp
        src/wbc_capability_panel.cpp
        src/switch_button.cpp
)

# Include directories
include_directories(include)

# Create library
add_library(${PROJECT_NAME} SHARED ${MOC_FILES} ${SRC_FILES})

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_link_libraries(${PROJECT_NAME} PUBLIC
  Qt5::Widgets
  ${rclcpp_TARGETS}
  rviz_common::rviz_common
  arms_controller_common::arms_controller_common
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  tf2::tf2
  ${arms_ros2_control_msgs_TARGETS}
)

# Export the plugin
ament_export_dependencies(rclcpp)
ament_export_dependencies(rviz_common)

# Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include
  DESTINATION include
)

# 使用宏 pluginlib_export_plugin_description_file 导出插件描述文件
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
