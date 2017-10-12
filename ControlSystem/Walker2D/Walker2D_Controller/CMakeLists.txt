FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_library (${CTRL_SYS}_Controller SHARED ${headers} ${sources})

target_link_libraries (${CTRL_SYS}_Controller SJutils SJfilters SJrbdl ${CTRL_SYS}_model)
