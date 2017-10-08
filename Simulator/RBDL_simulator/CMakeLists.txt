include_directories(${PROJECT_SOURCE_DIR}/Simulator/RBDL_simulator)

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/Simulator/RBDL_simulator/RBDL_Sim_config.h.cmake ${PROJECT_SOURCE_DIR}/Simulator/RBDL_simulator/RBDL_Sim_Configuration.h)

add_subdirectory(SimManager)
add_subdirectory(Systems/${CTRL_SYS})

add_executable (simulation_RBDL main.cpp)
target_link_libraries (simulation_RBDL
  ${CTRL_SYS}_Sim_System
)
