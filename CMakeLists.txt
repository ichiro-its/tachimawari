cmake_minimum_required(VERSION 3.5)
project(tachimawari)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(jitsuyo REQUIRED)
find_package(kansei_interfaces REQUIRED)
find_package(keisan REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tachimawari_interfaces REQUIRED)

# TODO: Enable me later.
# find_package(musen REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  "src/${PROJECT_NAME}/control/controller/module/cm740.cpp"
  "src/${PROJECT_NAME}/control/controller/packet/protocol_1/instruction/bulk_read_packet.cpp"
  "src/${PROJECT_NAME}/control/controller/packet/protocol_1/instruction/read_packet.cpp"
  "src/${PROJECT_NAME}/control/controller/packet/protocol_1/instruction/sync_write_packet.cpp"
  "src/${PROJECT_NAME}/control/controller/packet/protocol_1/instruction/write_packet.cpp"
  "src/${PROJECT_NAME}/control/controller/packet/protocol_1/model/packet.cpp"
  "src/${PROJECT_NAME}/control/controller/packet/protocol_1/status/bulk_read_data.cpp"
  "src/${PROJECT_NAME}/control/controller/packet/protocol_1/status/status_packet.cpp"
  "src/${PROJECT_NAME}/control/controller/packet/protocol_1/utils/word.cpp"
  "src/${PROJECT_NAME}/control/controller/platform/linux.cpp"
  "src/${PROJECT_NAME}/control/controller/utils/timer.cpp"
  "src/${PROJECT_NAME}/control/manager/control_manager.cpp"
  "src/${PROJECT_NAME}/control/node/control_node.cpp"
  "src/${PROJECT_NAME}/control/sdk/module/dynamixel_sdk.cpp"
  "src/${PROJECT_NAME}/control/sdk/packet/model/group_bulk_read.cpp"
  "src/${PROJECT_NAME}/control/sdk/packet/protocol_1/group_bulk_read.cpp"
  "src/${PROJECT_NAME}/control/sdk/packet/protocol_1/group_sync_write.cpp"
  "src/${PROJECT_NAME}/control/sdk/packet/protocol_2/group_bulk_read.cpp"
  "src/${PROJECT_NAME}/control/sdk/packet/protocol_2/group_sync_write.cpp"
  "src/${PROJECT_NAME}/imu/node/imu_node.cpp"
  "src/${PROJECT_NAME}/imu/node/imu_provider.cpp"
  "src/${PROJECT_NAME}/joint/model/joint_id.cpp"
  "src/${PROJECT_NAME}/joint/model/joint.cpp"
  "src/${PROJECT_NAME}/joint/node/joint_manager.cpp"
  "src/${PROJECT_NAME}/joint/node/joint_node.cpp"
  "src/${PROJECT_NAME}/joint/utils/middleware.cpp"
  "src/${PROJECT_NAME}/joint/utils/node_control.cpp"
  "src/${PROJECT_NAME}/node/tachimawari_node.cpp"
  # TODO: Enable me later.
  # "src/${PROJECT_NAME}/node/rviz_server_node.cpp"
  # "src/${PROJECT_NAME}/node/rviz_client_node.cpp"
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  dynamixel_sdk
  jitsuyo
  kansei_interfaces
  keisan
  rclcpp
  tachimawari_interfaces
  tf2_ros
  sensor_msgs
  geometry_msgs)

install(DIRECTORY "include" DESTINATION ".")

install(DIRECTORY
  "launch"
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  "urdf"
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  "rviz"
  DESTINATION share/${PROJECT_NAME}/
)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin")

add_executable(check_joints "src/check_joints_main.cpp")
target_include_directories(check_joints PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(check_joints ${PROJECT_NAME})
add_executable(init_joints "src/init_joints_main.cpp")
target_include_directories(init_joints PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(init_joints ${PROJECT_NAME})

add_executable(main "src/tachimawari_main.cpp")
target_include_directories(main PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(main ${PROJECT_NAME})

add_executable(read_buttons "src/read_buttons_main.cpp")
target_include_directories(read_buttons PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(read_buttons ${PROJECT_NAME})

add_executable(read_joints "src/read_joints_main.cpp")
target_include_directories(read_joints PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(read_joints ${PROJECT_NAME})

# TODO: Enable me later.
# add_executable(rviz_server "src/rviz_server_main.cpp")
# target_include_directories(rviz_server PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_link_libraries(${PROJECT_NAME} musen)
# target_link_libraries(rviz_server ${PROJECT_NAME})
#
# add_executable(rviz_client "src/rviz_client_main.cpp")
# target_include_directories(rviz_client PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_link_libraries(rviz_client ${PROJECT_NAME})

install(TARGETS
  check_joints
  init_joints
  main
  read_buttons
  read_joints
  # TODO: Enable me later.
  # rviz_server
  # rviz_client
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_tests
  "test/joint/joint_test.cpp")

  target_include_directories(${PROJECT_NAME}_tests PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

  target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})
  # TODO: Enable me later.
  # ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  dynamixel_sdk
  jitsuyo
  kansei_interfaces
  keisan
  rclcpp
  tf2_ros
  tachimawari_interfaces)
ament_export_include_directories("include")
ament_export_libraries(${PROJECT_NAME})
ament_package()
