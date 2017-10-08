FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_library (Openchain3DoF_Controller SHARED ${headers} ${sources})

target_link_libraries (Openchain3DoF_Controller SJutils SJfilters SJrbdl OC3_model)
