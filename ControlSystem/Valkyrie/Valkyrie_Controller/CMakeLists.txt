FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE sources *.cpp)

add_library (Valkyrie_Controller SHARED ${headers} ${sources})

target_link_libraries (Valkyrie_Controller SJutils SJfilters SJrbdl Valkyrie_model WBLC SJplanner)
