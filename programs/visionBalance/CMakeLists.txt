cmake_dependent_option(ENABLE_visionBalance "Enable/disable visionBalance program" ON
                       ENABLE_TravisLib OFF)

if(ENABLE_visionBalance)

    add_executable(visionBalance main.cpp
                                 VisionBalance.hpp
                                 VisionBalance.cpp
                                 SegmentorThread.hpp
                                 SegmentorThread.cpp)

    target_link_libraries(visionBalance YARP::YARP_OS
                                        YARP::YARP_init
                                        YARP::YARP_dev
                                        YARP::YARP_sig
                                        TravisLib)

    install(TARGETS visionBalance
            DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_visionBalance OFF CACHE BOOL "Enable/disable visionBalance program" FORCE)

endif()
