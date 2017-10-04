## Reference:
## https://github.com/tc/opengl-to-video-sample

FILE(GLOB_RECURSE header *.h)
file(GLOB_RECURSE source *.cpp)
add_library (opengl_recorder ${header} ${source} )

target_link_libraries(opengl_recorder "-framework GLUT")

find_package(GLEW REQUIRED)
if (GLEW_FOUND)
    include_directories(${GLEW_INCLUDE_DIRS}/)
    link_libraries(${GLEW_LIBRARIES})
endif()

if(WIN32)
  set(OpenCV_DIR "d:/libs/opencv-2.4.8/build")
else()
  set(OpenCV_DIR "/usr/lib/opencv")
endif()
# find_package(OpenCV) # REQUIRED COMPONENTS core imgproc)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(opengl_recorder ${OPENGL_LIBRARIES})
target_link_libraries(opengl_recorder ${OpenCV_LIBS}) # opencv_highgui)



