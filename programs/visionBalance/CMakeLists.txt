cmake_dependent_option(ENABLE_visionBalance "Enable/disable visionBalance program" ON
                       ENABLE_TravisLib OFF)

if(ENABLE_visionBalance)


# look for libfreenect
find_path(LIBFREENECT_INCLUDE_DIRS
   NAMES libfreenect.h
   PATH_SUFFIXES libfreenect)
if("${LIBFREENECT_INCLUDE_DIRS}" STREQUAL "LIBFREENECT_INCLUDE_DIRS-NOTFOUND")
   set(LIBFREENECT_EXAMPLE_MESSAGE "Disabled - libfreenect.h not found" PARENT_SCOPE)
else()
   find_library(LIBFREENECT_LIBS freenect HINTS /usr/local/lib64)
   if("${LIBFREENECT_LIBS}" STREQUAL "LIBFREENECT_LIBS-NOTFOUND")
       set(LIBFREENECT_EXAMPLE_MESSAGE "Disabled - libfreenect not found" PARENT_SCOPE)
   else()
       set(LIBFREENECT_EXAMPLE_MESSAGE "Enabled" PARENT_SCOPE)
   endif()
endif()

include_directories(${LIBFREENECT_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})


    add_executable(visionBalance main.cpp
                                 VisionBalance.hpp
                                 VisionBalance.cpp
				 data_capture.cpp
		                 data_capture.hpp
			         SegmentorThread.cpp
				 SegmentorThread.hpp
				 )


    target_link_libraries(visionBalance YARP::YARP_OS
                                        YARP::YARP_init
                                        YARP::YARP_dev
                                        YARP::YARP_sig
                                        ${OpenCV_LIBS}
                                        ${LIBFREENECT_LIBS}
                                        fovis
                                        TravisLib)

    install(TARGETS visionBalance
            DESTINATION ${CMAKE_INSTALL_BINDIR})

else()

    set(ENABLE_visionBalance OFF CACHE BOOL "Enable/disable visionBalance program" FORCE)

endif()
