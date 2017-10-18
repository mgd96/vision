// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// Thanks to https://github.com/ros-perception/image_transport_plugins

#include "RosImageTransport.hpp"

namespace roboticslab
{

using namespace cv;
using namespace std;

Mat depthToImage(Mat depth) {
	Mat image;

	image = Mat(depth.size(), CV_8UC1);
	depth.convertTo(image, CV_8U, -255/10.0, 255);  // pixel[x, y] = (-maxValue/maxDepth)*depth[x,y] + maxValue   

	return image;
}


Mat decodeImage(sensor_msgs_CompressedImage *message) {
	Mat image;
	
	image = imdecode(Mat(message->data), IMWRITE_JPEG_QUALITY);
	//cvtColor(image, image, COLOR_RGB2BGR);
	
	return image;
}


Mat decodeDepth(sensor_msgs_CompressedImage *message) {
	
	// variable for return
	Mat image;
	
	// get encoding
	string image_encoding = message->format.substr(0, message->format.find(';'));
	
	
	if (message->data.size() > sizeof(ConfigHeader)) {
	
		//read compression type from stream
		ConfigHeader compressionConfig;
		memcpy(&compressionConfig, &message->data[0], sizeof(compressionConfig));
	
		// get compressed image data
		const vector<uint8_t> imageData(message->data.begin() + sizeof(compressionConfig), message->data.end());
	
		// depth map decoding
		float depthQuantA, depthQuantB;
		depthQuantA = compressionConfig.depthParam[0];
		depthQuantB = compressionConfig.depthParam[1];
		
		// if bitDepth(image_encoding) == 32
		Mat decompressed;
		
		decompressed = imdecode(imageData, CV_LOAD_IMAGE_UNCHANGED);

		size_t rows = decompressed.rows;
		size_t cols = decompressed.cols;

		if ((rows > 0) && (cols > 0)) {
			image = Mat(rows, cols, CV_32FC1);

			// Depth conversion
			MatIterator_<float> itDepthImg = image.begin<float>(),
						    itDepthImg_end = image.end<float>();
			MatConstIterator_<unsigned short> itInvDepthImg = decompressed.begin<unsigned short>(),
						                  itInvDepthImg_end = decompressed.end<unsigned short>();

			for (; (itDepthImg != itDepthImg_end) && (itInvDepthImg != itInvDepthImg_end); ++itDepthImg, ++itInvDepthImg) {
				// check for NaN & max depth
				if (*itInvDepthImg) {
					*itDepthImg = depthQuantA / ((float)*itInvDepthImg - depthQuantB);
				}
				else {
					*itDepthImg = std::numeric_limits<float>::quiet_NaN();
					//*itDepthImg = 0.0;
				}
			}
		}
		
		return image;
		
	}
}

}  // namespace roboticslab

