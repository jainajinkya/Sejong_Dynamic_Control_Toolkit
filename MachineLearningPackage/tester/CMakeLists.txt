FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE sources *.cpp)

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

add_executable (learning_study ${headers} ${sources} )
target_link_libraries(learning_study SJutils)
target_link_libraries(learning_study environment)
target_link_libraries(learning_study agent)
target_link_libraries(learning_study planner)
target_link_libraries(learning_study ${PYTHON_LIBRARIES})
