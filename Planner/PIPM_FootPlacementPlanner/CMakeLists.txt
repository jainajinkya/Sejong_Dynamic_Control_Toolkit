FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE cppheaders *.hpp)
FILE(GLOB_RECURSE sources *.cpp)
add_library(PIPM_planner SHARED ${sources} ${headers} ${cppheaders})

target_link_libraries(PIPM_planner SJutils)
