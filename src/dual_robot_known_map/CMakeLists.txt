cmake_minimum_required(VERSION 3.8)
project(dual_robot_known_map)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz worlds maps
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/prefix_odom_tf.py
  scripts/prefix_scan.py
  scripts/peer_scan_relay.py
  scripts/peer_tf_relay.py
  scripts/drive_four_demo.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
