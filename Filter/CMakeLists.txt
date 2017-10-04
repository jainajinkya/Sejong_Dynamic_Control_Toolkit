FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE sources *.cpp)
add_library(SJfilters SHARED ${sources} ${headers})
target_link_libraries(SJfilters SJutils)