FILE(GLOB_RECURSE headers *.hh)
FILE(GLOB_RECURSE sources *.cc)
add_library (SJGoldfarb SHARED ${headers} ${sources})

install(TARGETS SJGoldfarb DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${headers} DESTINATION "${INSTALL_INCLUDE_DIR}/Optimizer/Goldfarb")
