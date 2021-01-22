cmake_minimum_required(VERSION 3.0.2)
project(kalman_filter)

find_package(catkin REQUIRED
	roscpp
	std_msgs
	keypoint_3d_matching_msgs
	cv_bridge
    pcl_ros
)

find_package(OpenCV REQUIRED)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME} 
	CATKIN_DEPENDS roscpp std_msgs keypoint_3d_matching_msgs cv_bridge pcl_ros
)


include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

add_executable(kalman_openpose3D src/main_openpose.cpp src/kalman_openpose3D.cpp src/utilities.cpp)
target_link_libraries(kalman_openpose3D ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(kalman_points src/main_points.cpp src/kalman_points.cpp)
target_link_libraries(kalman_points ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})