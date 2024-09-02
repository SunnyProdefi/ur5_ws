cmake_minimum_required(VERSION 3.10)
project(ur5_control)


find_package(catkin REQUIRED COMPONENTS
  kdl_parser
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  sensor_msgs
  trac_ik_lib
  trajectory_msgs
)

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
message(STATUS "Pinocchio Libraries: ${pinocchio_LIBRARIES}")
find_package(urdfdom REQUIRED)
find_package(Boost COMPONENTS filesystem serialization system REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur5_control
 CATKIN_DEPENDS 
 kdl_parser 
 roscpp 
 rospy 
 sensor_msgs 
 std_msgs 
 trac_ik_lib 
 trajectory_msgs 
 std_srvs 
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${urdfdom_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories(${PINOCCHIO_LIBRARY_DIRS})
add_executable(dynamics_and_control src/dynamics_and_control.cpp)
target_link_libraries(dynamics_and_control 
  ${catkin_LIBRARIES} 
  ${EIGEN3_LIBRARIES} 
  ${pinocchio_LIBRARIES}
  /opt/openrobots/lib/libpinocchio_default.so 
  /opt/openrobots/lib/libpinocchio_collision.so 
  /opt/openrobots/lib/libpinocchio_parsers.so 
  ${urdfdom_LIBRARIES}
  ${Boost_LIBRARIES}
)
add_dependencies(dynamics_and_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )


add_executable(joint_dynamics_calculator src/joint_dynamics_calculator.cpp)
target_link_libraries(joint_dynamics_calculator 
  ${catkin_LIBRARIES} 
  ${EIGEN3_LIBRARIES} 
  ${pinocchio_LIBRARIES}
  /opt/openrobots/lib/libpinocchio_default.so 
  /opt/openrobots/lib/libpinocchio_collision.so 
  /opt/openrobots/lib/libpinocchio_parsers.so 
  ${urdfdom_LIBRARIES}
  ${Boost_LIBRARIES}
)
add_dependencies(joint_dynamics_calculator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

add_executable(ur5_set_zero_position src/ur5_set_zero_position.cpp)
target_link_libraries(ur5_set_zero_position 
  ${catkin_LIBRARIES} 
  ${EIGEN3_LIBRARIES} 
  ${pinocchio_LIBRARIES}
  /opt/openrobots/lib/libpinocchio_default.so 
  /opt/openrobots/lib/libpinocchio_collision.so 
  /opt/openrobots/lib/libpinocchio_parsers.so 
  ${urdfdom_LIBRARIES}
  ${Boost_LIBRARIES}
)
add_dependencies(ur5_set_zero_position ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )

catkin_install_python(PROGRAMS
  scripts/trajectory_planner.py
  scripts/call_trajectory_planner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)