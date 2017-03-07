cmake_minimum_required(VERSION 2.4.6)
project(chessboard-tracker)


find_package(catkin REQUIRED
	roscpp std_msgs sensor_msgs image_transport cv_bridge eigen_conversions
	cmake_modules)

find_package(OpenCV REQUIRED)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ar_image_capture
  CATKIN_DEPENDS roscpp
  #DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
	${catkin_INCLUDE_DIRS}
  	${OpenCV_INCLUDE_DIRS}
  	)



add_library(fast_checkerboard_detector_nodelet src/fast_checkerboard_detector_nodelet.cpp src/fast_checkerboard_detector.cpp)

add_executable(fast_checkerboard_detector
				nodes/fast_checkerboard_detector_node.cpp
				src/fast_checkerboard_detector.cpp)

add_dependencies(fast_checkerboard_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(fast_checkerboard_detector
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )
