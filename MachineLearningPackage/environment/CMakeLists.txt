FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE sources *.cpp)

add_library (environment SHARED ${headers} ${sources} )
target_link_libraries(environment SJutils)
