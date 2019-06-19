// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SegmentorThread.hpp"

namespace roboticslab
{

/************************************************************************/
void SegmentorThread::setIRGBDSensor(yarp::dev::IRGBDSensor *_iRGBDSensor) {
    iRGBDSensor = _iRGBDSensor;
}

/************************************************************************/
void SegmentorThread::setOutImg(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > * _pOutImg) {
    pOutImg = _pOutImg;
}

/************************************************************************/
void SegmentorThread::setOutPort(yarp::os::Port * _pOutPort) {
    pOutPort = _pOutPort;
}


/************************************************************************/
void SegmentorThread::init(yarp::os::ResourceFinder &rf) {

yarp::os::Property rgbIntrinsicParams;
    yarp::os::Property depthIntrinsicParams;

    iRGBDSensor->getRgbIntrinsicParam(rgbIntrinsicParams);
    iRGBDSensor->getDepthIntrinsicParam(depthIntrinsicParams);

    fx_d = depthIntrinsicParams.find("focalLengthX").asFloat64();
    fy_d = depthIntrinsicParams.find("focalLengthY").asFloat64();
    cx_d = depthIntrinsicParams.find("principalPointX").asFloat64();
    cy_d = depthIntrinsicParams.find("principalPointY").asFloat64();

    fx_rgb = rgbIntrinsicParams.find("focalLengthX").asFloat64();
    fy_rgb = rgbIntrinsicParams.find("focalLengthY").asFloat64();
    cx_rgb = rgbIntrinsicParams.find("principalPointX").asFloat64();
    cy_rgb = rgbIntrinsicParams.find("principalPointY").asFloat64();

    algorithm = DEFAULT_ALGORITHM;
    locate = DEFAULT_LOCATE;
    maxNumBlobs = DEFAULT_MAX_NUM_BLOBS;
    morphClosing = DEFAULT_MORPH_CLOSING;
    morphOpening = DEFAULT_MORPH_OPENING;
    outImage = DEFAULT_OUT_IMAGE;
    outFeatures.fromString(DEFAULT_OUT_FEATURES);  // it's a bottle!!
    outFeaturesFormat = DEFAULT_OUT_FEATURES_FORMAT;
    int rateMs = DEFAULT_RATE_MS;
    seeBounding = DEFAULT_SEE_BOUNDING;
    threshold = DEFAULT_THRESHOLD;

    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("SegmentorThread options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--algorithm (default: \"%s\")\n",algorithm.c_str());
        printf("\t--locate (centroid or bottom; default: \"%s\")\n",locate.c_str());
        printf("\t--maxNumBlobs (default: \"%d\")\n",maxNumBlobs);
        printf("\t--morphClosing (percentage, 2 or 4 okay; default: \"%f\")\n",morphClosing);
        printf("\t--morphOpening (percentage, 2 or 4 okay; default: \"%f\")\n",morphOpening);
        printf("\t--outFeatures (mmX,mmY,mmZ,pxXpos,pxYpos,pxX,pxY,angle,area,aspectRatio,rectangularity,axisFirst,axisSecond \
solidity,hue,sat,val,hueStdDev,satStdDev,valStdDev,time; \
default: \"(%s)\")\n",outFeatures.toString().c_str());
        printf("\t--outFeaturesFormat (0=bottled,1=minimal; default: \"%d\")\n",outFeaturesFormat);
        printf("\t--outImage (0=rgb,1=bin; default: \"%d\")\n",outImage);
        printf("\t--rateMs (default: \"%d\")\n",rateMs);
        printf("\t--seeBounding (0=none,1=box,2=contour,3=both; default: \"%d\")\n",seeBounding);
        printf("\t--threshold (default: \"%d\")\n",threshold);
        // Do not exit: let last layer exit so we get help from the complete chain.
    }

    if (rf.check("algorithm")) algorithm = rf.find("algorithm").asString();
    if (rf.check("locate")) locate = rf.find("locate").asString();
    if (rf.check("maxNumBlobs")) maxNumBlobs = rf.find("maxNumBlobs").asInt32();
    if (rf.check("morphClosing")) morphClosing = rf.find("morphClosing").asFloat64();
    if (rf.check("morphOpening")) morphOpening = rf.find("morphOpening").asFloat64();
    if (rf.check("outFeaturesFormat")) outFeaturesFormat = rf.find("outFeaturesFormat").asInt32();

    printf("SegmentorThread using fx_d: %f, fy_d: %f, cx_d: %f, cy_d: %f.\n",
        fx_d,fy_d,cx_d,cy_d);
    printf("SegmentorThread using fx_rgb: %f, fy_rgb: %f, cx_rgb: %f, cy_rgb: %f.\n",
        fx_rgb,fy_rgb,cx_rgb,cy_rgb);
    printf("SegmentorThread using algorithm: %s, locate: %s.\n",
        algorithm.c_str(),locate.c_str());
    printf("SegmentorThread using maxNumBlobs: %d, morphClosing: %.2f, outFeaturesFormat: %d.\n",
        maxNumBlobs,morphClosing,outFeaturesFormat);

    if (rf.check("outFeatures")) {
        outFeatures = *(rf.find("outFeatures").asList());  // simple overrride
    }
    printf("SegmentorThread using outFeatures: (%s).\n", outFeatures.toString().c_str());

    if (rf.check("outImage")) outImage = rf.find("outImage").asInt32();
    if (rf.check("rateMs")) rateMs = rf.find("rateMs").asInt32();
    if (rf.check("threshold")) threshold = rf.find("threshold").asInt32();
    if (rf.check("seeBounding")) seeBounding = rf.find("seeBounding").asInt32();
    printf("SegmentorThread using outImage: %d, rateMs: %d, seeBounding: %d, threshold: %d.\n",
        outImage, rateMs, seeBounding, threshold);

    printf("--------------------------------------------------------------\n");
    if(rf.check("help")) {
        ::exit(1);
    }

    if(cropSelector != 0) {
        processor.reset();
        inCropSelectorPort->setReader(processor);
    }

    // Wait for the first few frames to arrive. We kept receiving invalid pixel codes
    // from the depthCamera device if started straight away.
    yarp::os::Time::delay(1);

    this->setPeriod(rateMs * 0.001);
    this->start();
}

/************************************************************************/
void SegmentorThread::run() {

     
        printf("entro en run");
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
      yarp::os::Bottle output_angles;

      odom->processFrame(cap->getGrayImage(), cap->getDepthImage());

      // get the integrated pose estimate.
      Eigen::Isometry3d cam_to_local = odom->getPose();

      // get the motion estimate for this frame to the previous frame.
      Eigen::Isometry3d motion_estimate = odom->getMotionEstimate();

      // display the motion estimate.  These values are all given in the RGB
      // camera frame, where +Z is forward, +X points right, +Y points down, and
      // the origin is located at the focal point of the RGB camera.

      std::cout << isometryToString(cam_to_local,output_angles) << "\n";

      /*if (output_angles.size() > 0)
          pOutPort->write(output_angles);*/


    }

    printf("Shutting down\n");
    cap->stopDataCapture();
    delete odom;
    delete cap;





 }

}  // namespace roboticslab
