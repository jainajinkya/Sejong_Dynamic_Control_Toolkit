FILE(GLOB_RECURSE header *.hpp)
file(GLOB_RECURSE source *.cpp)
add_library (${CTRL_SYS}_Sim_System ${header} ${source} )

target_link_libraries (${CTRL_SYS}_Sim_System
  SJutils
  SJrbdl
  SJMobyLCP
  RBDL_SimManager
  ${CTRL_SYS}_Controller
)
