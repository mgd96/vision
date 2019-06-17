// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __VISION_BALANCE_HPP__
#define __VISION_BALANCE_HPP__

#include "SegmentorThread.hpp"

#define DEFAULT_CROP_SELECTOR 0  // 1=true
#define DEFAULT_RGBD_DEVICE "RGBDSensorClient"
#define DEFAULT_RGBD_LOCAL "/visionBalance"
#define DEFAULT_RGBD_REMOTE "/rgbd"
#define DEFAULT_WATCHDOG    2       // [s]


namespace roboticslab
{

/**
 * @ingroup visionBalance
 *
 * @brief Computer Vision 1.
 */
class VisionBalance : public yarp::os::RFModule {
  private:
    SegmentorThread segmentorThread;
    //
    yarp::dev::PolyDriver dd;
    yarp::dev::IRGBDSensor *iRGBDSensor;

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

#endif  // __VISION_BALANCE_HPP__

