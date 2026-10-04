cmake_minimum_required(VERSION 3.8)
project(dual_robot_known_map)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/prefix_odom_tf.py
  scripts/prefix_scan.py
  scripts/peer_scan_relay.py
  scripts/peer_tf_relay.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
