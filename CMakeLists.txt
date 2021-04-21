cmake_minimum_required(VERSION 2.8.3)
project(pff_sem)


find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  geometry_msgs
  tf
  laser_geometry
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(velodyne_people src/velodyne_people.cpp)
target_link_libraries(velodyne_people ${catkin_LIBRARIES})
add_executable(velodyne_doors src/velodyne_doors.cpp)
target_link_libraries(velodyne_doors ${catkin_LIBRARIES})
add_executable(velodyne_chairs src/velodyne_chairs.cpp)
target_link_libraries(velodyne_chairs ${catkin_LIBRARIES})