# Copyright: (C) 2017 Universidad Carlos III de Madrid
# Authors: Juan G. Victores & Raul de Santos Rico
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

option(ENABLE_YarpPlugins "Enable/disable YarpPlugins" OFF)

if(ENABLE_YarpPlugins)

# Create device interface target.
#add_library(VisionInterfaces INTERFACE)

# Configure usage requirements.
#target_include_directories(VisionInterfaces INTERFACE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
#                                                      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# Install interface headers.
#install(FILES IWhatever.h
#        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Register export set.
#install(TARGETS VisionInterfaces
#        EXPORT ROBOTICSLAB_VISION)

# Export variables to the global scope.
#set_property(GLOBAL APPEND PROPERTY _exported_targets VisionInterfaces)

# YARP devices.
#add_subdirectory(...)

endif()
