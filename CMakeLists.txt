################################################
# Title: Sejong Dynamic Control Toolkit
################################################

cmake_minimum_required(VERSION 3.0)
project(SDC_Toolkit)

#------------------------------------------------------------------------
#     Build Setup
#------------------------------------------------------------------------
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_CXX_FLAGS "-std=c++11")

include_directories ("${PROJECT_SOURCE_DIR}/")
include_directories ("${PROJECT_SOURCE_DIR}/ExternalSource")
include_directories ("${PROJECT_SOURCE_DIR}/WBC")
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# set (CMAKE_BUILD_TYPE Debug)
set (CMAKE_BUILD_TYPE Release)
add_definitions(-O3)

#------------------------------------------------------------------------
#     System Selection
#------------------------------------------------------------------------
# SET (CTRL_SYS "Valkyrie")
# SET (CTRL_SYS "Hume")
# SET (CTRL_SYS "NAO")
# SET (CTRL_SYS "DracoP1")
# SET (CTRL_SYS "DracoP1Rot")
SET (CTRL_SYS "Mercury")
# SET (CTRL_SYS "Openchain3DoF")
# SET (CTRL_SYS "Openchain2DoF")
# SET (CTRL_SYS "Walker2D")

include_directories ("${PROJECT_SOURCE_DIR}/ControlSystem/${CTRL_SYS}")

#------------------------------------------------------------------------
#     Simulator Selection
#------------------------------------------------------------------------
SET (RECORD_SIMULATION 0)
add_subdirectory (Simulator/srSimulator)
# add_subdirectory (Simulator/RBDL_simulator)

#------------------------------------------------------------------------
#    Directory Selection
#------------------------------------------------------------------------
add_subdirectory (Utils)
add_subdirectory (Filter)
add_subdirectory (ExternalSource)
add_subdirectory (Optimizer)
add_subdirectory (WBC/WBDC)
add_subdirectory (ControlSystem/${CTRL_SYS})
# add_subdirectory (MachineLearning_Package)
# add_subdirectory (LocomotionPlanner)
add_subdirectory(Tests) # For unit testing with googletest

#------------------------------------------------------------------------
# Configuration
#------------------------------------------------------------------------
SET (THIS_COM "${PROJECT_SOURCE_DIR}/" )
SET (DEFINITION_FILE "ControlSystem/${CTRL_SYS}/${CTRL_SYS}_Definition.h")
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/config.h.cmake ${PROJECT_SOURCE_DIR}/Configuration.h)


#------------------------------------------------------------------------
# Printout
#------------------------------------------------------------------------
message ("")
message ("-[ MAKE INFORMATION ]----------------------------------------")
message (" The project name: ${PROJECT_NAME}")
message (" The version: ${${PROJECT_NAME}_VERSION}")
message (" The project source directory: ${PROJECT_SOURCE_DIR}")
message (" The project binary directory: ${PROJECT_BINARY_DIR}")
message (" The install destination: ${CMAKE_INSTALL_PREFIX}")
message (" The control system: ${CTRL_SYS}")
message ("--------------------------------------------------------------")
message ("")


