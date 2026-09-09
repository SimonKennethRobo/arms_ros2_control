cmake_minimum_required(VERSION 3.14)
project(adaptive_gripper_controller)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
        controller_interface
        hardware_interface
        pluginlib
        rclcpp
        rclcpp_lifecycle
)
find_package(ament_cmake REQUIRED)
foreach (Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach ()


add_library(${PROJECT_NAME} SHARED
        src/adaptive_gripper_controller.cpp
)
# GCC 9 (Focal) defaults to gnu++14; std::optional/std::clamp need C++17.
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        PRIVATE
        src)
# Foxy's ros2_control (0.11) exports no CMake targets, so ${controller_interface_TARGETS}
# and ${hardware_interface_TARGETS} are empty; ament_target_dependencies covers the
# old ament_export_libraries style.
ament_target_dependencies(${PROJECT_NAME} PUBLIC
        controller_interface
        hardware_interface
)

target_link_libraries(${PROJECT_NAME} PUBLIC
        ${pluginlib_TARGETS}
        ${rclcpp_TARGETS}
        ${rclcpp_lifecycle_TARGETS}
)

pluginlib_export_plugin_description_file(controller_interface ros_control_plugins.xml)

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION bin
)

install(
        DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
