# cmake needs this line
cmake_minimum_required(VERSION 3.1)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Define project name
project(opencv_audio_video)
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

include_directories(./cam_lib ./logger)
add_subdirectory(./cam_lib)
add_subdirectory(./logger)
#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/cam_lib)
add_executable(cam_exe main.cpp)

target_link_libraries(cam_exe logger av_cam)
install(TARGETS cam_exe DESTINATION bin)