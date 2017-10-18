if(ENABLE_RosImageTransport)

    # Set the name for the library
    set(LIBNAME RosImageTransport)

    set(SOURCES RosImageTransport.cpp RosImageTransport.hpp)

    find_package(OpenCV REQUIRED)

    include_directories(${OpenCV_INCLUDE_DIRS}  ${ROS_MESSAGES_INCLUDE_DIRS})

    add_library(${LIBNAME} ${SOURCES})

    target_link_libraries(${LIBNAME} ${OpenCV_LIBS})

    install(TARGETS RosImageTransport
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin)

    set(ROBOTICSLAB_VISION_INCLUDE_DIRS ${ROBOTICSLAB_VISION_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} CACHE INTERNAL "appended header dirs")
    set(ROBOTICSLAB_VISION_LIBRARIES ${ROBOTICSLAB_VISION_LIBRARIES} ${LIBNAME} CACHE INTERNAL "appended libraries")

endif()
