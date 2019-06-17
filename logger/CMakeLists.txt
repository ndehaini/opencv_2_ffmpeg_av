cmake_minimum_required (VERSION 2.6)
project (LOGGER)

set(LOGGER_VERSION_MAJOR 1)
set(LOGGER_VERSION_MINOR 0)

#set(SOURCE_FILES json_lib fileio.c c_str_io.c BCM_2835_Wrapper.c command_process.c ServerSocket.c)
set(SOURCE_FILES logger.c)
add_library(logger SHARED ${SOURCE_FILES})

#TARGET_LINK_LIBRARIES(rpi_util pthread bcm2835)
