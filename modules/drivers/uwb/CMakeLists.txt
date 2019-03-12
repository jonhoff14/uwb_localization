cmake_minimum_required(VERSION 2.8.3)
project(uwb_driver)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    tf2_ros
)

catkin_package(
   LIBRARIES ${catkin_LIBRARIES} roscpp
   CATKIN_DEPENDS roscpp std_msgs tf2_ros
   DEPENDS  protobuf
)

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "-g -pipe -W -Wall -Wno-sign-compare -Wno-unused-parameter -fPIC -O2 -Wno-parentheses -rdynamic -std=c++11")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

##find_package(Proj)
if(DEFINED PROTOBUF_LIBRARIES)
    MESSAGE(STATUS "proto: " ${PROTOBUF_LIBRARIES})
else()
    set(PROTOBUF_LIBRARIES "protobuf")
    MESSAGE(STATUS "proto: " ${PROTOBUF_LIBRARIES})
endif()

set(MODULE_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../")
set(COMMON_PROTO_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/common/proto/")
include_directories(${MODULE_ROOT_DIR})
include_directories(${COMMON_PROTO_INCLUDE_DIR})

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
link_directories(lib)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Save protobuf file paths as variables
set(ERROR_CODE_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/common/proto/error_code.pb.cc")
set(HEADER_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/common/proto/header.pb.cc")
set(GEOMETRY_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/common/proto/geometry.pb.cc")
set(PNC_POINT_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/common/proto/pnc_point.pb.cc")

set(LOCALIZATION_GPS_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/localization/proto/gps.pb.cc")
set(LOCALIZATION_IMU_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/localization/proto/imu.pb.cc")
set(LOCALIZATION_LOCALIZATION_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/localization/proto/localization.pb.cc")
set(LOCALIZATION_POSE_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/localization/proto/pose.pb.cc")

set(GNSSBESTPOSE_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/drivers/gnss/proto/gnss_best_pose.pb.cc")
#set(IMU_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/drivers/gnss/proto/imu.pb.cc")

set(UWB_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/proto/config.pb.cc")

#FILE(GLOB UWB_PB_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.pb.cc")

# Create utils library with protobuf dependencies for ROS nodes
add_library(utils_uwb src/util/utils.cpp
  ${UWB_PB_SRCS} ${LOCALIZATION_GPS_PB_SRCS} ${LOCALIZATION_IMU_PB_SRCS}
  ${LOCALIZATION_LOCALIZATION_PB_SRCS} ${LOCALIZATION_POSE_PB_SRCS} ${GNSSBESTPOSE_PB_SRCS}
  ${ERROR_CODE_SRCS} ${HEADER_PB_SRCS} ${GEOMETRY_PB_SRCS} ${PNC_POINT_PB_SRCS})
target_link_libraries(utils_uwb ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})

# Compile UWB driver that reads serial data
add_executable(${PROJECT_NAME} src/uwb/uwb_driver.cpp)
target_link_libraries(${PROJECT_NAME} utils_uwb ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})

# Compile UWB-GNSS fusion node that collects messages and fuses them
add_executable(uwb_gnss_fusion src/uwb_gnss_fusion/uwb_gnss_fusion.cpp src/uwb_gnss_fusion/coordinate_transform.cpp)
target_link_libraries(uwb_gnss_fusion utils_uwb ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} uwb_gnss_fusion utils_uwb
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY conf/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/conf)
