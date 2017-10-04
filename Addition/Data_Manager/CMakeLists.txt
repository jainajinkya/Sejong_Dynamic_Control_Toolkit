project (Supervisor)
cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

set (${PROJECT_NAME}_VERSION_NAME "star")
set (${PROJECT_NAME}_VERSION_MAJOR 1)
set (${PROJECT_NAME}_VERSION_MINOR 0)
set (${PROJECT_NAME}_VERSION_PATCH 0)
set (${PROJECT_NAME}_VERSION ${${PROJECT_NAME}_VERSION_MAJOR}.${${PROJECT_NAME}_VERSION_MINOR}.${${PROJECT_NAME}_VERSION_PATCH})
set (CMAKE_BUILD_TYPE Debug)
message ("")
message ("-[ MAKE INFORMATION ]-----------------------------------------------------------")
message (" The project name: ${PROJECT_NAME}")
message (" The version: ${${PROJECT_NAME}_VERSION}")
message (" The project source directory: ${PROJECT_SOURCE_DIR}")
message (" The project binary directory: ${PROJECT_BINARY_DIR}")
message (" The install destination: ${CMAKE_INSTALL_PREFIX}")
message ("--------------------------------------------------------------------------------")
message ("")

include_directories ("${PROJECT_SOURCE_DIR}/../../utils")
include_directories ("${PROJECT_SOURCE_DIR}/../../ExternalSources")
include_directories ("${PROJECT_SOURCE_DIR}/../../")

#find_package(Eigen3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})


set (headers 
DataSave.h
data_protocol.h
../../utils/Sejong_Thread.h
../../utils/wrap_eigen.hpp
../../utils/comm_udp.h
../../utils/utilities.h
../../utils/DataManager.h
)

set (sources
DataSave.cpp
Status_Display.cpp
../../utils/Sejong_Thread.cpp
../../utils/wrap_eigen.cpp
../../utils/comm_udp.cpp
../../utils/utilities.cpp
../../utils/DataManager.cpp
)

link_directories("/usr/local/lib/")
link_directories("/usr/lib/")

add_executable(Status_Display ${sources} ${headers})
add_dependencies(Status_Display comm_comm)
target_link_libraries(Status_Display pthread)
