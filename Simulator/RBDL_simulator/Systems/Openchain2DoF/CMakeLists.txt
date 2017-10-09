FILE(GLOB_RECURSE header *.hpp)
file(GLOB_RECURSE source *.cpp)
add_library (Openchain2DoF_Sim_System ${header} ${source} )

target_link_libraries (Openchain2DoF_Sim_System
  SJutils
  SJrbdl
  RBDL_SimManager
  ${CTRL_SYS}_Controller
)
