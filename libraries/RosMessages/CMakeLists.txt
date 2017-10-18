# Copyright: UC3M 2014 (C)
# Authors: Dennis Pinto Rivero, Juan G. Victores

# reduce warning level with cmake 2.6
cmake_minimum_required(VERSION 2.6)
#cmake policies
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(LIBNAME RosMessages)
set(HEADERS sensor_msgs_CompressedImage.h std_msgs_Header.h TickTime.h)

set(ROS_MESSAGES_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended header dirs")

# Generate complete routes to header files (not used currently)
foreach(f ${HEADERS})
    set(LIBFILES ${LIBFILES} ${CMAKE_CURRENT_SOURCE_DIR}/${f})    
endforeach()
