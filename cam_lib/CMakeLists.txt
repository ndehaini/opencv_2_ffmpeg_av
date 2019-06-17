# cmake needs this line
cmake_minimum_required(VERSION 3.1)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

#include_directories(../logger)
# Define project name
project(opencv_audio_video)
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)

# If the package has been found, several variables will
# be set, you can find the full list with descriptions
# in the OpenCVConfig.cmake file.
# Print some message showing some of them
message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

# Declare the executable target built from your sources
#add_executable(opencv_example example.cpp)
add_library(av_cam SHARED cam_capture.cpp cam_exception.cpp ffmpg_mux.cpp)

# Link your application with OpenCV libraries
target_link_libraries(av_cam logger SDL2 swresample avformat avcodec avfilter swscale z m avutil asound ${OpenCV_LIBS})
install(TARGETS av_cam DESTINATION lib)
install(FILES cam_capture.h cam_exception.h ffmpg_mux.h DESTINATION include)
#swresample avformat avcodec avfilter swscale m avutil SDL2 ${OpenCV_LIBS}