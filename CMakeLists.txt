cmake_minimum_required(VERSION 2.8.3)
project(pcl)

set(CMAKE_CXX_STANDARD 14)
###################################
find_package(
	catkin REQUIRED COMPONENTS 
	roscpp
	rospy
	std_msgs
	pcl_conversions
	pcl_ros
	#nodelet
	message_generation
	image_transport
	image_geometry
	cv_bridge
	)
	
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

############################ service
add_service_files(FILES ComputeRMSE.srv)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES pointcloud_localize
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

############################ include
include_directories(
${PCL_INCLUDE_DIRS}
 include
 lib
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)

############################ libraries
add_library(CloudDetection src/CloudDetection.cpp)
target_link_libraries(CloudDetection ${catkin_LIBRARIES})
#add_dependencies(republish_cxx ${{PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(CascadeDetection src/CascadeDetection.cpp)
target_link_libraries(CascadeDetection ${catkin_LIBRARIES})

add_library(KalmanFilter src/KalmanFilter.cpp)
target_link_libraries(KalmanFilter ${catkin_LIBRARIES})

add_library(PointcloudFilter src/PointcloudFilter.cpp)
target_link_libraries(PointcloudFilter ${catkin_LIBRARIES})

add_library(recognition src/recognition.cpp)
target_link_libraries(recognition ${catkin_LIBRARIES})

add_library(Utility src/Utility.cpp)
target_link_libraries(Utility ${catkin_LIBRARIES})


############################ executables
add_executable(cloud_detection_node 
	src/cloud_detection_node.cpp
)
target_link_libraries(cloud_detection_node 
	${PCL_LIBRARIES} 
	${catkin_LIBRARIES} 
	CloudDetection 
	KalmanFilter 
	PointcloudFilter
	recognition
	Utility
)

add_executable(cascade_detection_node 
	src/cascade_detection_node.cpp
)
target_link_libraries(cascade_detection_node 
	PRIVATE ${OpenCV_LIBS}
	CascadeDetection
	Utility
)

add_executable(pub_cmd src/pub_cmd.cpp)
target_link_libraries(pub_cmd ${catkin_LIBRARIES})



############################ other executables 
add_executable(region_growing_rgb_segmentation 
	src/region_growing_rgb_segmentation.cpp
)
target_link_libraries(region_growing_rgb_segmentation
	${catkin_LIBRARIES}
	${PCL_LIBRARIES}
)

add_executable(eigenTest 
	src/eigenTest.cpp
)
target_link_libraries(eigenTest
	${catkin_LIBRARIES}
)



add_executable(bluerov_detect 
	src/bluerov_detect.cpp
)
target_link_libraries(bluerov_detect 
	PRIVATE ${OpenCV_LIBS}
)

add_executable(bluerov_nestdetect 
	src/bluerov_nestdetect.cpp
)
target_link_libraries(bluerov_nestdetect 
	PRIVATE ${OpenCV_LIBS}
)

#add_executable(recognition_node src/recognition_node.cpp)
#target_link_libraries(recognition_node ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(tracking src/tracking.cpp)
#target_link_libraries(tracking ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(tracking_sample src/tracking_sample.cpp)
#target_link_libraries(tracking_sample ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(correspondence_grouping src/correspondence_grouping.cpp)
#target_link_libraries(correspondence_grouping ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(global_hypothesis_verification src/global_hypothesis_verification.cpp)
#target_link_libraries(global_hypothesis_verification ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(cloud_centroid src/cloud_centroid.cpp)
#target_link_libraries(cloud_centroid ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(rangeimage src/rangeimage.cpp)
#target_link_libraries(rangeimage ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(icp src/icp.cpp)
#target_link_libraries(icp ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(icp_pose src/icp_pose.cpp)
#target_link_libraries(icp_pose ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_executable(republish src/republish.cpp)
#target_link_libraries(republish ${PCL_LIBRARIES} ${catkin_LIBRARIES})

#add_library(pcl_nodelet src/pcl_nodelet.cpp)
#target_link_libraries(pcl_nodelet ${catkin_EXPORTED_LIBRARIES})

