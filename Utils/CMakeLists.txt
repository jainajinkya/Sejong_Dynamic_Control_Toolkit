FILE(GLOB_RECURSE cppheaders *.hpp)
FILE(GLOB_RECURSE sources *.cpp)
include_directories ("${PROJECT_SOURCE_DIR}/ExternalSource")

add_library(SJutils SHARED ${sources} ${headers} ${cppheaders})

find_package(Threads REQUIRED)

if(THREADS_HAVE_PTHREAD_ARG)
  target_compile_options(SJutils PUBLIC "-pthread")
endif()
if(CMAKE_THREAD_LIBS_INIT)
  target_link_libraries(SJutils "${CMAKE_THREAD_LIBS_INIT}")
endif()
