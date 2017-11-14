FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_library (SJparam_handler SHARED ${headers} ${sources})
target_link_libraries (SJparam_handler yaml-cpp SJutils)

install(TARGETS SJparam_handler DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION "${INSTALL_INCLUDE_DIR}/ParamHandler")
