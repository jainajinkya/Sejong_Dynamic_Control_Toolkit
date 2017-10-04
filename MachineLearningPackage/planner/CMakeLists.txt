FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE sources *.cpp)

add_library (planner SHARED ${headers} ${sources} )
target_link_libraries(planner SJutils)
target_link_libraries(planner SJplanner)
