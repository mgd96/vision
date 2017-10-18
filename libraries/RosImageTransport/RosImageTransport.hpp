// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __ROS_IMAGE_TRANSPORT_HPP__
#define __ROS_IMAGE_TRANSPORT_HPP__

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include "sensor_msgs_CompressedImage.h"

/**
 * @ingroup vision_libraries
 * @defgroup RosImageTransport
 */

namespace roboticslab
{

enum compressionFormat { UNDEFINED = -1, INV_DEPTH };

struct ConfigHeader {
	compressionFormat format;
	float depthParam[2];
};

cv::Mat depthToImage(cv::Mat depth);


cv::Mat decodeImage(sensor_msgs_CompressedImage *message);


cv::Mat decodeDepth(sensor_msgs_CompressedImage *message);

}  // namespace roboticslab

#endif  // __ROS_IMAGE_TRANSPORT_HPP__

