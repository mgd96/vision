// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "VisionBalance.hpp"

#include <cstdio>
#include <cstring>
#include <iostream>

#include <yarp/os/Bottle.h>

namespace roboticslab
{

/************************************************************************/

VisionBalance::VisionBalance()  {}

/************************************************************************/
bool VisionBalance::configure(yarp::os::ResourceFinder &rf) {

    outPort.open("/visionBalance/state:o");
    segmentorThread.setOutPort(&outPort);
    segmentorThread.init(rf);
    return true;

}

/*****************************************************************/
double VisionBalance::getPeriod() {
    return 0.02;  // [s]
}

/************************************************************************/

bool VisionBalance::updateModule() {
   
    //std::printf("VisionBalance update module...\n");
    return true;
    
}
/************************************************************************/

bool VisionBalance::interruptModule() {
    std::printf("VisionBalance interrupting...\n");
    return true;
}

/************************************************************************/

bool VisionBalance::close() {
    std::printf("VisionBalance closing...\n");
    outPort.close();
    return true;
}

/************************************************************************/

}  // namespace roboticslab
