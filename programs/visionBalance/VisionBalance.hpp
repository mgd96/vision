// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __VISION_BALANCE_HPP__
#define __VISION_BALANCE_HPP__

#include "SegmentorThread.hpp"

#include <yarp/os/Port.h>
#include <yarp/os/RFModule.h>

#include <fovis/fovis.hpp>

#include "data_capture.hpp"

namespace roboticslab
{

/**
 * @ingroup visionBalance
 *
 * @brief Computer Vision 1.
 */
class VisionBalance : public yarp::os::RFModule {
  public:
    VisionBalance();
    
    bool configure(yarp::os::ResourceFinder &rf);
    bool interruptModule();
    double getPeriod();
    bool updateModule();
    bool close();

  private:
    yarp::os::Port outPort;
    SegmentorThread segmentorThread;
    
};

}  // namespace roboticslab

#endif  // __VISION_BALANCE_HPP__
