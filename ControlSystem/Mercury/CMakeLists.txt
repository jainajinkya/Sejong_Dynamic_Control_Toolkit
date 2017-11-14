add_subdirectory(${CTRL_SYS}_Controller)
add_subdirectory(Robot_Model)
install(FILES Mercury_Definition.h DESTINATION "${INSTALL_INCLUDE_DIR}/")
