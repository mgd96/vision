// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "VisionBalance.hpp"

namespace roboticslab
{

/************************************************************************/
bool VisionBalance::configure(yarp::os::ResourceFinder &rf) {
    std::string strRGBDLocal  = DEFAULT_RGBD_LOCAL;
    std::string strRGBDRemote = DEFAULT_RGBD_REMOTE;
    fprintf(stdout,"-----------------------\n");

    if (!rf.check("help")) {
        yarp::os::Property options;
        options.fromString( rf.toString() );  //-- Should get noMirror, noRGBMirror, noDepthMirror, video modes...
        options.put("localRpcPort",strRGBDLocal+"/rpc:o");
        options.put("remoteRpcPort",strRGBDRemote+"/rpc:i");
        printf("llego aquí");       
        printf("llego aquí_2"); 
    }

    printf("llego aquí");
    segmentorThread.init(rf);
    printf("llego aquí");

    //-----------------OPEN LOCAL PORTS------------//
   outImg.open(strRGBDLocal + "/img:o");
   outPort.open(strRGBDLocal + "/state:o");
 
    return true;
}

/*****************************************************************/
double VisionBalance::getPeriod() {
    return watchdog;  // [s]
}

/************************************************************************/

bool VisionBalance::updateModule() {
    printf("VisionBalance alive...\n");
    return true;
}

/************************************************************************/

bool VisionBalance::interruptModule() {
    printf("VisionBalance closing...\n");
    segmentorThread.stop();
    outImg.interrupt();
    outPort.interrupt();
   // dd.close();
    outImg.close();
    outPort.close();
}

/************************************************************************/

}  // namespace roboticslab
