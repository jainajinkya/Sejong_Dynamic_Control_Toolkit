FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_library (Openchain2DoF_Controller SHARED ${headers} ${sources})

target_link_libraries (Openchain2DoF_Controller SJutils SJfilters SJrbdl OC2_model)
