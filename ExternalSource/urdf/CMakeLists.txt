FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE sources *.cpp)
add_library(SJurdf SHARED ${sources} ${headers})
