FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_library(Valkyrie_model SHARED ${sources} ${headers})

target_link_libraries(Valkyrie_model SJutils)
target_link_libraries(Valkyrie_model SJrbdl)
