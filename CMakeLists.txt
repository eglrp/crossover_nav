cmake_minimum_required(VERSION 2.8.3)
project(crossover_nav)
# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
# View the makefile commands during build
#set(CMAKE_VERBOSE_MAKEFILE on)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


# Find Eigen3 and G2O
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules/")
message("\n\n\n\n\n modules : ${CMAKE_MODULE_PATH}")
FIND_PACKAGE(Eigen3 REQUIRED)
#FIND_PACKAGE(G2O REQUIRED)
#FIND_PACKAGE(SuiteSparse REQUIRED)
#include_directories(${G2O_INCLUDE_DIRS})
#link_directories(${G2O_LIBRARY_DIRS})
##link_libraries(${G2O_LIBRARIES})
#message("G2o-libraries " ${G2O_LIBRARIES})
#SET (LIBS_LINK cholmod)
#find_package(Cholmod REQUIRED)
#message("----------------${CHOLMOD_FOUND}----------------\n\n\n")
#message("----------------${CSPARSE_INCLUDE_DIR}----------------\n\n\n")
#find_package(Chofldmod REQUIRED)
# ceres solver from google
#find_package(Ceres REQUIRED)
#include_directories(${CERES_INCLUDE_DIRS})


#ROS package
find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  std_msgs
  visualization_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  sensor_fusion_comm
  message_generation
  librealsense
)
## Generate messages in the 'msg' folder
add_message_files(
  FILES 
  Navdata.msg
  Ack.msg
  Info.msg
  odom_data.msg 
  Status.msg
 )
## Generate added messages and services with any dependencies listed here
generate_messages (
   DEPENDENCIES
   geometry_msgs
   std_msgs
 )
catkin_package( 
  CATKIN_DEPENDS message_runtime sensor_fusion_comm
)
include_directories("/usr/include/eigen3")

add_executable(offb_test_cpp src/offb_node.cpp)
target_link_libraries(offb_test_cpp ${catkin_LIBRARIES})

# add_executable(thirdorder src/Ulink_capter_and_inertial_nav.cpp)
# target_link_libraries(thirdorder ${catkin_LIBRARIES})

#add_executable(gps_tester2 src/gps_tester.cpp)
#target_link_libraries(gps_tester2 ${catkin_LIBRARIES} ${LIBS_LINK})

#add_executable(simulate_optimize_gps src/simulate_optimize_gps.cpp)
#target_link_libraries(simulate_optimize_gps ${catkin_LIBRARIES} ${LIBS_LINK})

#include_directories("src/tutorial_slam2d")
#add_executable(tutorials src/tutorial_slam2d/tutorial_slam2d.cpp)
#target_link_libraries(tutorials ${catkin_LIBRARIES} ${G2O_LIBRARY_DIRS} ${SUITEPARSE_LIBRARY_DIRS})

add_executable(crossover src/crossover_node.cpp)
target_link_libraries(crossover ${catkin_LIBRARIES})

add_executable(test_tf src/test_tf/test_tf.cpp)
target_link_libraries(test_tf ${catkin_LIBRARIES})


#lms
  #for matlabplot and csvphaser

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})



#add_executable(lms src/lms/lms_node2.cc)
#target_link_libraries(lms ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${PYTHON_LIBRARIES})

#add_executable(lms_real src/lms/lms_real.cc)
#target_link_libraries(lms_real ${catkin_LIBRARIES} ${CERES_LIBRARIES} ${PYTHON_LIBRARIES})
