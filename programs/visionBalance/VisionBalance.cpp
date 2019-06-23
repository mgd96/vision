// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "VisionBalance.hpp"

#include <cstdio>
#include <cstring>
#include <iostream>

#include <yarp/os/Bottle.h>

namespace
{
    char * isometryToString(const Eigen::Isometry3d& m, yarp::os::Bottle &output_angles)
    {

      char *result = (char *) malloc(sizeof(char) * 3);

      std::memset(result, 0, sizeof(result));
      Eigen::Vector3d xyz = m.translation();
      Eigen::Vector3d rpy = m.rotation().eulerAngles(0, 1, 2);
      std::snprintf(result, 79, "%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f %6.2f");

      double angx=(rpy(0) * 180/M_PI);
      double angy=(rpy(1) * 180/M_PI);
      double angz=(rpy(2) * 180/M_PI);

      if (angx>100)  angx=angx-180;
      if (angy>100)  angy=angy-180;
      if (angz>100)  angz=angz-180;

      if (angx<-100)  angx=angx+180;
      if (angy<-100)  angy=angy+180;
      if (angz<-100)  angz=angz+180;

      result[0]=angx;
      result[1]=angy;
      result[2]=angz;

      output_angles.addDouble(angx);
      output_angles.addDouble(angy);
      output_angles.addDouble(angz);

      std::cout<<"ang x= "<<angx<<" ang y= "<<angz<<" ang z= "<<angy<<std::endl;

      return result;
    }
}

namespace roboticslab
{

/************************************************************************/

VisionBalance::VisionBalance() : cap(0), odom(0) {}

/************************************************************************/
bool VisionBalance::configure(yarp::os::ResourceFinder &rf) {
    // initialize the device
    /*cap = new fovis_example::DataCapture();

    if (!cap->initialize()) {
        std::fprintf(stderr, "Unable to initialize Kinect sensor\n");
    }

    if (!cap->startDataCapture()) {
        std::fprintf(stderr, "Unable to start data capture\n");
    }

    // get the RGB camera parameters of our device
    fovis::Rectification rect(cap->getRgbParameters());

    fovis::VisualOdometryOptions options =
        fovis::VisualOdometry::getDefaultOptions();
    // If we wanted to play around with the different VO parameters, we could set
    // them here in the "options" variable.

    // setup the visual odometry
    odom = new fovis::VisualOdometry(&rect, options);*/

    outPort.open("/visionBalance/state:o");

    return true;
}

/*****************************************************************/
double VisionBalance::getPeriod() {
    return 0.02;  // [s]
}

/************************************************************************/

bool VisionBalance::updateModule() {
    bool nothing;
    std::printf("VisionBalance update module...\n");

    cap = new fovis_example::DataCapture();
       
    if (!cap->initialize()) {
        std::fprintf(stderr, "Unable to initialize Kinect sensor\n");
    }
    //cap->initialize();
    //cap->startDataCapture();
    printf("LLEGO AQI 2");
    if (!cap->startDataCapture()) {
        std::fprintf(stderr, "Unable to start data capture\n");
        //printf("LLEGO AQI 2");
    }
       printf("LLEGO AQI 3");
    // get the RGB camera parameters of our device
    fovis::Rectification rect(cap->getRgbParameters());

    fovis::VisualOdometryOptions options =
        fovis::VisualOdometry::getDefaultOptions();
    // If we wanted to play around with the different VO parameters, we could set
    // them here in the "options" variable.

    // setup the visual odometry
    odom = new fovis::VisualOdometry(&rect, options);

    while (cap->captureOne()) {
        /*if (!cap->captureOne()) {
            std::fprintf(stderr, "Capture failed\n");
            return true;
        }*/

        yarp::os::Bottle output_angles;
        const uint8_t* gray = cap->getGrayImage();
        fovis::DepthImage* depth = cap->getDepthImage();
        odom->processFrame(gray, depth);

        // get the integrated pose estimate.
        Eigen::Isometry3d cam_to_local = odom->getPose();

        // get the motion estimate for this frame to the previous frame.
        Eigen::Isometry3d motion_estimate = odom->getMotionEstimate();

        // display the motion estimate.  These values are all given in the RGB
        // camera frame, where +Z is forward, +X points right, +Y points down, and
        // the origin is located at the focal point of the RGB camera.

        std::cout << isometryToString(cam_to_local, output_angles) << "\n";

        if (output_angles.size() > 0) {
            outPort.write(output_angles);
        }
    }

    std::fprintf(stderr, "Capture failed\n");

    if (odom) {
        delete odom;
        odom = 0;
    }

    if (cap) {
        delete cap;
        cap = 0;
    }

    return true;
}
/************************************************************************/

bool VisionBalance::interruptModule() {
    std::printf("VisionBalance interrupting...\n");
    cap->stopDataCapture();
    return true;
}

/************************************************************************/

bool VisionBalance::close() {
    std::printf("VisionBalance closing...\n");

    if (odom) {
        delete odom;
        odom = 0;
    }

    if (cap) {
        delete cap;
        cap = 0;
    }

    outPort.close();
    return true;
}

/************************************************************************/

}  // namespace roboticslab
