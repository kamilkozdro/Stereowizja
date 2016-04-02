#include <iostream>
#include <opencv2\opencv.hpp>
#include "StereoCalibration.h"
#include "StereoVision.h"

using namespace cv;

int main()
{
	
	/*
	CStereoCalibration calibrate;

	if (calibrate.openCameras(1, 2) != 1)
		return 0;
	calibrate.runCalibration();
	calibrate.saveSettings("testKalibracji.yml");
	std::cout << calibrate.error_rms << endl;
	*/
	
	CStereoVision stereoVision;
	Mat depth;
	namedWindow("leftCam");
	namedWindow("rightCam");
	namedWindow("depth");
	stereoVision.initStereoVision("testKalibracji.yml", 1, 2);
	while (waitKey(10) == -1)
	{
		stereoVision.grabFrames();
		stereoVision.undistortRectifyFrames(stereoVision.leftFrame, stereoVision.rightFrame);
		//stereoVision.drawParallerLines(stereoVision.leftFrame);
		//stereoVision.drawParallerLines(stereoVision.rightFrame);
		stereoVision.calcDisparityMap();
		depth = stereoVision.reproject();
		imshow("leftCam", stereoVision.leftFrame);
		imshow("rightCam", stereoVision.rightFrame);
		imshow("depth", depth);
	}
	
	return 1;
}