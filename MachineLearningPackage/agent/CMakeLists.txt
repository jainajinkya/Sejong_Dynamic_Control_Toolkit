FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE sources *.cpp)

add_library (agent SHARED ${headers} ${sources} )
target_link_libraries(agent SJutils)
target_link_libraries(agent environment)
