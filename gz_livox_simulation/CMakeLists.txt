cmake_minimum_required(VERSION 3.16)
project(gz_livox_simulation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(sdformat_vendor REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(sdformat14 REQUIRED)

add_library(gz_livox_raycast_system SHARED src/livox_raycast_system.cpp)
target_include_directories(gz_livox_raycast_system PRIVATE include)
target_link_libraries(gz_livox_raycast_system gz-plugin2::gz-plugin2 gz-sim8::gz-sim8)
ament_target_dependencies(gz_livox_raycast_system
  ament_index_cpp
  livox_ros_driver2
  rclcpp
  sensor_msgs)

add_library(gz_holonomic_pose_system SHARED src/holonomic_pose_system.cpp)
target_link_libraries(gz_holonomic_pose_system
  gz-plugin2::gz-plugin2
  gz-msgs10::gz-msgs10
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13)

add_executable(test_scan_pattern test/test_scan_pattern.cpp)
target_include_directories(test_scan_pattern PRIVATE include)
enable_testing()
add_test(NAME scan_pattern_regression COMMAND test_scan_pattern)
add_executable(test_holonomic_drive test/test_holonomic_drive.cpp)
target_link_libraries(test_holonomic_drive
  gz-sim8::gz-sim8 gz-transport13::gz-transport13 gz-msgs10::gz-msgs10)
add_test(NAME holonomic_drive_regression COMMAND test_holonomic_drive
  ${CMAKE_CURRENT_SOURCE_DIR}/models/sentry_robot/model.sdf)
set_tests_properties(holonomic_drive_regression PROPERTIES
  ENVIRONMENT "GZ_SIM_SYSTEM_PLUGIN_PATH=$<TARGET_FILE_DIR:gz_holonomic_pose_system>;GZ_PARTITION=sentry_holonomic_regression"
  TIMEOUT 60)
ament_add_pytest_test(gazebo_sim_migration test/test_gazebo_sim_migration.py)
ament_add_pytest_test(sentry_model_contract test/test_sentry_model.py)
ament_add_pytest_test(static_transform_contract test/test_static_transforms.py)

install(TARGETS gz_livox_raycast_system gz_holonomic_pose_system DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY models worlds launch DESTINATION share/${PROJECT_NAME})
ament_package()
