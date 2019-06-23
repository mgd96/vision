// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SegmentorThread.hpp"

namespace roboticslab
{
 yarp::os::Bottle output_angles;

/************************************************************************/
void SegmentorThread::setOutPort(yarp::os::Port * _pOutPort) {
    pOutPort = _pOutPort;
}
/************************************************************************/
void SegmentorThread::init(yarp::os::ResourceFinder &rf) {
      printf("entro en init");
    // Wait for the first few frames to arrive. We kept receiving invalid pixel codes
    // from the depthCamera device if started straight away.


    yarp::os::Time::delay(1);
    this->setPeriod(2 * 0.1);
    this->start();
}

/************************************************************************/
void SegmentorThread::run() {

     
        fprintf(stdout,"entro en run-------\n");
    // initialize the device
    fovis_example::DataCapture* cap = new fovis_example::DataCapture();
    if(!cap->initialize()) {
      fprintf(stderr, "Unable to initialize Kinect sensor\n");

    }
    if(!cap->startDataCapture()) {
      fprintf(stderr, "Unable to start data capture\n");

    }

    // get the RGB camera parameters of our device
    fovis::Rectification rect(cap->getRgbParameters());

    fovis::VisualOdometryOptions options =
        fovis::VisualOdometry::getDefaultOptions();
    // If we wanted to play around with the different VO parameters, we could set
    // them here in the "options" variable.

    // setup the visual odometry
    fovis::VisualOdometry* odom = new fovis::VisualOdometry(&rect, options);

    // exit cleanly on CTL-C
    struct sigaction new_action;

    //new_action.sa_sigaction = sig_action;

    sigemptyset(&new_action.sa_mask);
    new_action.sa_flags = 0;
    sigaction(SIGINT, &new_action, NULL);
    sigaction(SIGTERM, &new_action, NULL);
    sigaction(SIGHUP, &new_action, NULL);

    while(!shutdown_flag) {
      if(!cap->captureOne()) {
        fprintf(stderr, "Capture failed\n");
        break;
      }
     
    
      odom->processFrame(cap->getGrayImage(), cap->getDepthImage());
      // get the integrated pose estimate.
      Eigen::Isometry3d cam_to_local = odom->getPose();
      // get the motion estimate for this frame to the previous frame.
      Eigen::Isometry3d motion_estimate = odom->getMotionEstimate();
      // display the motion estimate.  These values are all given in the RGB
      // camera frame, where +Z is forward, +X points right, +Y points down, and
      // the origin is located at the focal point of the RGB camera.
       isometryToString(cam_to_local,output_angles);

     if (output_angles.size() > 0)
          pOutPort->write(output_angles);


    }
    printf("Shutting down\n");
    cap->stopDataCapture();
    delete odom;
    delete cap;
   




 }

}  // namespace roboticslab
