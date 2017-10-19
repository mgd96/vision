// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __HAAR_DETECTION_HPP__
#define __HAAR_DETECTION_HPP__

#include "SegmentorThread.hpp"

#define DEFAULT_CROP_SELECTOR 1  // 1=true

#define DEFAULT_IMAGE_TOPIC     "/xtion/rgb/image_raw/compressed"
#define DEFAULT_DEPTH_TOPIC     "/xtion/depth_registered/image_raw/compressedDepth"
#define DEFAULT_PORT_NAMESPACE  "/haarCascadeClassifier"
#define DEFAULT_IMAGEOUT_PORT   (DEFAULT_PORT_NAMESPACE "/image:o")
#define DEFAULT_STATEOUT_PORT   (DEFAULT_PORT_NAMESPACE "/state:o")
#define DEFAULT_CROP_IMAGEOUT_PORT  (DEFAULT_PORT_NAMESPACE "/cropSelector/image:o")
#define DEFAULT_CROP_STATEIN_PORT   (DEFAULT_PORT_NAMESPACE "/cropSelector/state:i")

#define DEFAULT_WATCHDOG    2       // [s]


namespace roboticslab
{

/**
 * @ingroup haarDetection
 *
 * @brief Computer Vision segment faces.
 */
class HaarDetection : public yarp::os::RFModule {
  private:
    SegmentorThread segmentorThread;

    // Ports to get source data (from ROS topics)
    yarp::os::Subscriber<Image_t> inImagePort;
    yarp::os::Subscriber<DepthImage_t> inDepthPort;

    // Output YARP ports
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outImg;
    yarp::os::Port outPort;

    // Crop selector YARP ports
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outCropSelectorImg;
    yarp::os::Port inCropSelectorPort;

    int cropSelector;

    bool interruptModule();
    double getPeriod();
    bool updateModule();
    double watchdog;

  public:
    bool configure(yarp::os::ResourceFinder &rf);
};

}  // namespace roboticslab

#endif  // __HAAR_DETECTION_HPP__

