FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE sources *.cpp)

add_library(WBDC SHARED ${sources} ${headers})
target_link_libraries(WBDC SJutils SJgurobi SJGoldfarb)
