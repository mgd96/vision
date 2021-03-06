// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * 
 * @ingroup vision_programs
 * \defgroup kinectPxToReal kinectPxToReal
 *
 * kinectPxToReal means Pixel to Homogeneous Transformation Matrix for Kinect
 *
 * <b> Legal </b>
 *
 * Copyright:  2012 (C) Universidad Carlos III de Madrid
 *
 * Author: <a href="http://roboticslab.uc3m.es/roboticslab/persona_publ.php?id_pers=72">Juan G. Victores</a>
 *
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see repos/asibot-main/doc/LGPL.TXT
 *
 */

#include "KinectPxToReal.hpp"

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

int main(int argc, char *argv[]) {
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("kinectPxToReal/conf");
    rf.setDefaultConfigFile("kinectPxToReal.ini");
    rf.configure(argc, argv);

    KinectPxToReal mod;
    if(rf.check("help")) {
        return mod.runModule(rf);
    }

    printf("Run \"kinectPxToReal --help\" for options.\n");
    printf("kinectPxToReal checking for yarp network... ");
    fflush(stdout);
    yarp::os::Network yarp;
    if (!yarp::os::Network::checkNetwork()) {
        fprintf(stderr, "[fail]\nkinectPxToReal found no yarp network (try running \"yarpserver &\"), bye!\n");
        return 1;
    } else printf("[ok]\n");

    return mod.runModule(rf);
}

