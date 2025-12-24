cmake_minimum_required(VERSION 3.10)
project(actor_pose_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(gz_sim_vendor REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz_math_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-math7 REQUIRED)

add_library(actor_pose_ros2 SHARED src/actor_pose_ros2.cpp)
target_compile_features(actor_pose_ros2 PUBLIC cxx_std_17)

ament_target_dependencies(actor_pose_ros2
  rclcpp geometry_msgs tf2_ros rosgraph_msgs
)

target_link_libraries(actor_pose_ros2
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-math7::gz-math7
)
install(TARGETS actor_pose_ros2 LIBRARY DESTINATION lib)
# (helps some toolchains/IDEs pick up include dirs)
target_include_directories(actor_pose_ros2 PRIVATE
  $<TARGET_PROPERTY:gz-sim8::gz-sim8,INTERFACE_INCLUDE_DIRECTORIES>
  $<TARGET_PROPERTY:gz-plugin2::gz-plugin2,INTERFACE_INCLUDE_DIRECTORIES>
  $<TARGET_PROPERTY:gz-math7::gz-math7,INTERFACE_INCLUDE_DIRECTORIES>
)

add_library(model_waypoint_mover SHARED src/model_waypoint_mover.cpp)
target_compile_features(model_waypoint_mover PUBLIC cxx_std_17)
target_link_libraries(model_waypoint_mover
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-math7::gz-math7
)
install(TARGETS model_waypoint_mover
  LIBRARY DESTINATION lib)

ament_package()
