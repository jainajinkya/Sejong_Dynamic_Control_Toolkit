FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE sources *.cpp)
include_directories ("${PROJECT_SOURCE_DIR}/ControlSystem/${CTRL_SYS}/${CTRL_SYS}_Controller")

add_library (${CTRL_SYS}_Controller SHARED ${headers} ${sources})
target_link_libraries (${CTRL_SYS}_Controller SJutils SJfilters SJrbdl)
target_link_libraries (${CTRL_SYS}_Controller ${CTRL_SYS}_model)
target_link_libraries (${CTRL_SYS}_Controller WBDC)

