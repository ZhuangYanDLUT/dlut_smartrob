CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
project(dlut_hash_icp)
#set(CMAKE_BUILD_TYPE DEBUG)

#ROS
#=================================================================
find_package(catkin REQUIRED
  COMPONENTS roscpp rospy std_msgs message_generation tf pcl_ros)
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
#=================================================================

#cmake settings
#=================================================================
#set output library directory;
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#message(STATUS "this is LIBRARY_OUTPUT_PATH " ${LIBRARY_OUTPUT_PATH})

#set output binary directory;
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#message(STATUS "this is EXECUTABLE_OUTPUT_PATH " ${EXECUTABLE_OUTPUT_PATH})

#set runtime output directory;
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
#message(STATUS "this is CMAKE_RUNTIME_OUTPUT_DIRECTORY " ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

#set include files;
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)
message(STATUS "this is CMAKE_CURRENT_SOURCE_DIR " ${CMAKE_CURRENT_SOURCE_DIR})

#set source files;
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/ PROJECT_SOURCES)
AUX_SOURCE_DIRECTORY(src/src SRC_LIST)
#=================================================================

add_executable(dlut_hash_icp src/dlut_hash_icp.cpp ${SRC_LIST} ${PROJECT_HEADERS_MOC})
add_executable(dlut_dtbc src/dlut_dtbc.cpp ${SRC_LIST} ${PROJECT_HEADERS_MOC})
add_executable(dlut_align src/dlut_align.cpp ${SRC_LIST} ${PROJECT_HEADERS_MOC})
target_link_libraries(dlut_hash_icp
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )
target_link_libraries(dlut_dtbc
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )
target_link_libraries(dlut_align
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )
install(TARGETS dlut_hash_icp dlut_dtbc dlut_align
  ARCHIVE DESTINATION ~/catkin_ws/bin#dynamic lib, *.a;
  LIBRARY DESTINATION ~/catkin_ws/bin#static lib, *.so;
  RUNTIME DESTINATION ~/catkin_ws/bin#executable;
  )
