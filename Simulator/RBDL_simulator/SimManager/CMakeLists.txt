FILE(GLOB_RECURSE header *.hpp)
file(GLOB_RECURSE source *.cpp)

add_library(RBDL_SimManager ${header} ${source})
target_link_libraries (RBDL_SimManager
  ${PYTHON_LIBRARIES}
)
