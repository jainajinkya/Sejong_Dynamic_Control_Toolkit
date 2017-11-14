FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)
include_directories ("${PROJECT_SOURCE_DIR}/ControlSystem/${CTRL_SYS}/${CTRL_SYS}_Controller")

add_library (${CTRL_SYS}_Controller SHARED ${headers} ${sources})
target_link_libraries (${CTRL_SYS}_Controller SJutils SJfilters SJrbdl)
target_link_libraries (${CTRL_SYS}_Controller ${CTRL_SYS}_model)
target_link_libraries (${CTRL_SYS}_Controller WBDC)

target_link_libraries (${CTRL_SYS}_Controller SJparam_handler PIPM_planner)

install(TARGETS ${CTRL_SYS}_Controller DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION "${INSTALL_INCLUDE_DIR}/Mercury_Controller")
