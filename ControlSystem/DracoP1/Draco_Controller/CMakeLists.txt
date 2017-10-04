FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE sources *.cpp)

add_library (Draco_Controller SHARED ${headers} ${sources})

target_link_libraries (Draco_Controller SJutils SJfilters SJrbdl Draco_model)
target_link_libraries (Draco_Controller WBDC)
# target_link_libraries (Draco_Controller WBLC)
