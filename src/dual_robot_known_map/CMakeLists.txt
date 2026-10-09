cmake_minimum_required(VERSION 3.8)
project(dual_robot_known_map)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz worlds maps
  DESTINATION share/${PROJECT_NAME})
# (launch/ includes four_robots_pibt_cbba.launch.py)

install(PROGRAMS
  scripts/prefix_odom_tf.py
  scripts/prefix_scan.py
  scripts/peer_scan_relay.py
  scripts/peer_tf_relay.py
  scripts/drive_four_demo.py
  scripts/prioritized_mapf.py
  scripts/pibt_planner.py
  scripts/mapf_grid_params.py
  scripts/smart_adg.py
  scripts/adg_executor.py
  scripts/mrpa_executor.py
  scripts/cmd_vel_path_follower.py
  scripts/swarm_demo_goals.py
  scripts/cbba_nav_smoke_test.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
