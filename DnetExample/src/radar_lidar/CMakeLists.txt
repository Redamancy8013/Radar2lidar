CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(DnetExample)
set(CMAKE_CXX_FLAGS "-g3 -Wall -pthread")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common visualization io filters)
find_package(Pangolin REQUIRED)

SET(ALL_PROJECT_TOPLEVEL_PATH "/home/easycool/project/dnet/${PROJECT_NAME}/")
MESSAGE("ALL_PROJECT_TOPLEVEL_PATH: " ${ALL_PROJECT_TOPLEVEL_PATH})


include_directories(
include
 ${PCL_INCLUDE_DIRS} 
 ${ALL_PROJECT_TOPLEVEL_PATH}/include/
 ${EIGEN_ROOT}
 ${OpenCV_INCLUDE_DIRS}
 ${Pangolin_INCLUDE_DIRS}
 )

LINK_DIRECTORIES(
 ${ALL_PROJECT_TOPLEVEL_PATH}/lib/
 ${OpenCV_LIBRARY_DIRS}
 )

SET(EXECUTABLE_OUTPUT_PATH ${ALL_PROJECT_TOPLEVEL_PATH}/bin)

ADD_EXECUTABLE(radar_lidar
receive_main.cpp
receive_module.cpp
../common/timestamp.c)

TARGET_LINK_LIBRARIES(radar_lidar
dnet
${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${PCL_FILTERS_LIBRARIES}
${pcl_conversions_LIBRARIES}
pcl_visualization
${Boost_SYSTEM_LIBRARY}
${OpenCV_LIBS} 
${Pangolin_LIBRARIES} 
)
