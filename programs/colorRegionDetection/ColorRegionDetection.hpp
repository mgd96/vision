// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __COLOR_REGION_DETECTION_HPP__
#define __COLOR_REGION_DETECTION_HPP__

#include "SegmentorThread.hpp"

#define DEFAULT_CROP_SELECTOR 1  // 1=true

#define DEFAULT_IMAGE_TOPIC     "/xtion/rgb/image_raw"
#define DEFAULT_DEPTH_TOPIC     "/xtion/depth_registered/image_raw"
#define DEFAULT_PORT_NAMESPACE  "/colorRegionDetection"
#define DEFAULT_IMAGEOUT_PORT   (DEFAULT_PORT_NAMESPACE "/image:o")
#define DEFAULT_STATEOUT_PORT   (DEFAULT_PORT_NAMESPACE "/state:o")
#define DEFAULT_CROP_IMAGEOUT_PORT  (DEFAULT_PORT_NAMESPACE "/cropSelector/image:o")
#define DEFAULT_CROP_STATEIN_PORT   (DEFAULT_PORT_NAMESPACE "/cropSelector/state:i")

#define DEFAULT_WATCHDOG    2       // [s]


namespace roboticslab
{

/**
 * @ingroup colorRegionDetection
 *
 * @brief Computer Vision 1.
 */
class ColorRegionDetection : public yarp::os::RFModule {
  private:
    SegmentorThread segmentorThread;

    yarp::os::Subscriber<sensor_msgs_Image> inImagePort;
    yarp::os::Subscriber<sensor_msgs_Image> inDepthPort;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outImg;
    yarp::os::Port outPort;

    int cropSelector;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outCropSelectorImg;
    yarp::os::Port inCropSelectorPort;

    bool interruptModule();
    double getPeriod();
    bool updateModule();
    double watchdog;

  public:
    bool configure(yarp::os::ResourceFinder &rf);
};

}  // namespace roboticslab

#endif  // __COLOR_REGION_DETECTION_HPP__

