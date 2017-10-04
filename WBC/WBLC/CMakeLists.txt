FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE sources *.cpp)

add_library(WBLC SHARED ${sources} ${headers})
target_link_libraries(WBLC SJutils SJgurobi Polyhedron)
