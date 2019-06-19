// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "VisionBalance.hpp"

namespace roboticslab
{

/************************************************************************/
bool VisionBalance::configure(yarp::os::ResourceFinder &rf) {
       cropSelector = DEFAULT_CROP_SELECTOR;
    std::string strRGBDDevice = DEFAULT_RGBD_DEVICE;
    std::string strRGBDLocal = DEFAULT_RGBD_LOCAL;
    std::string strRGBDRemote = DEFAULT_RGBD_REMOTE;
    watchdog = DEFAULT_WATCHDOG;  // double

    fprintf(stdout,"--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("ColorRegionDetection options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--cropSelector (default: \"%d\")\n",cropSelector);
        printf("\t--RGBDDevice (device we create, default: \"%s\")\n",strRGBDDevice.c_str());
        printf("\t--RGBDLocal (if accesing remote, local port name, default: \"%s\")\n",strRGBDLocal.c_str());
        printf("\t--RGBDRemote (if accesing remote, remote port name, default: \"%s\")\n",strRGBDRemote.c_str());
        printf("\t--watchdog ([s] default: \"%f\")\n",watchdog);
        // Do not exit: let last layer exit so we get help from the complete chain.
    }
    if(rf.check("cropSelector")) cropSelector = rf.find("cropSelector").asInt32();
    printf("ColorRegionDetection using cropSelector: %d.\n",cropSelector);
    if(rf.check("RGBDDevice")) strRGBDDevice = rf.find("RGBDDevice").asString();
    if(rf.check("RGBDLocal")) strRGBDLocal = rf.find("RGBDLocal").asString();
    if(rf.check("RGBDRemote")) strRGBDRemote = rf.find("RGBDRemote").asString();
    if(rf.check("watchdog")) watchdog = rf.find("watchdog").asFloat64();
    printf("ColorRegionDetection using RGBDDevice: %s, RGBDLocal: %s, RGBDRemote: %s.\n",
        strRGBDDevice.c_str(), strRGBDLocal.c_str(), strRGBDRemote.c_str());
    printf("ColorRegionDetection using watchdog: %f.\n",watchdog);

    if (!rf.check("help")) {
        yarp::os::Property options;
        options.fromString( rf.toString() );  //-- Should get noMirror, noRGBMirror, noDepthMirror, video modes...
        options.put("device",strRGBDDevice);  //-- Important to override in case there is a "device" in the future
        options.put("localImagePort",strRGBDLocal+"/rgbImage:i");
        options.put("localDepthPort",strRGBDLocal+"/depthImage:i");
        options.put("localRpcPort",strRGBDLocal+"/rpc:o");
        options.put("remoteImagePort",strRGBDRemote+"/rgbImage:o");
        options.put("remoteDepthPort",strRGBDRemote+"/depthImage:o");
        options.put("remoteRpcPort",strRGBDRemote+"/rpc:i");
        //if(rf.check("noMirror")) options.put("noMirror",1);  //-- Replaced by options.fromString( rf.toString() );
        while(!dd.open(options)) {
            printf("Waiting for RGBDDevice \"%s\"...\n",strRGBDDevice.c_str());
            yarp::os::Time::delay(1);
        }
        printf("[ColorRegionDetection] success: RGBDDevice available.\n");
        if (! dd.view(iRGBDSensor) ) fprintf(stderr,"[ColorRegionDetection] warning: RGBDDevice bad view.\n");
        else printf("[ColorRegionDetection] success: RGBDDevice ok view.\n");

        segmentorThread.setIRGBDSensor(iRGBDSensor);
        segmentorThread.setOutImg(&outImg);
        segmentorThread.setOutPort(&outPort);

        segmentorThread.setCropSelector(cropSelector);
        if(cropSelector != 0) {
            segmentorThread.setOutCropSelectorImg(&outCropSelectorImg);
            segmentorThread.setInCropSelectorPort(&inCropSelectorPort);
        }
    }

    segmentorThread.init(rf);

    //-----------------OPEN LOCAL PORTS------------//
    outImg.open(strRGBDLocal + "/img:o");
    outPort.open(strRGBDLocal + "/state:o");
    if(cropSelector != 0) {
        outCropSelectorImg.open(strRGBDLocal + "/cropSelector/img:o");
        inCropSelectorPort.open(strRGBDLocal + "/cropSelector/state:i");
    }
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
