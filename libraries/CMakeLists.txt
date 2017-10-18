# Copyright: Universidad Carlos III de Madrid (C) 2013
# Authors: Juan G. Victores

set(COLOR_DEBUG_PART_OF_PROJECT TRUE)
set(ROS_MESSAGES_PART_OF_PROJECT TRUE)
add_subdirectory(ColorDebug) # ColorDebug sets COLOR_DEBUG_INCLUDE_DIRS
add_subdirectory(RosMessages) # RosMessages sets ROS_MESSAGES_INCLUDE_DIRS
set(ROBOTICSLAB_VISION_INCLUDE_DIRS ${ROBOTICSLAB_VISION_INCLUDE_DIRS} ${COLOR_DEBUG_INCLUDE_DIRS}  CACHE INTERNAL "appended header dirs")
set(ROBOTICSLAB_VISION_INCLUDE_DIRS ${ROBOTICSLAB_VISION_INCLUDE_DIRS} ${ROS_MESSAGES_INCLUDE_DIRS}  CACHE INTERNAL "appended header dirs")

add_subdirectory(TravisLib)
add_subdirectory(VisionYarp)  # VisionYarp a plugin library which contains YARP plugins
add_subdirectory(RosImageTransport)
