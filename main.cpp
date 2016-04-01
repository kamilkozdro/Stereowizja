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
	*/

	CStereoVision stereoVision;

	namedWindow("leftCam");
	namedWindow("rightCam");
	stereoVision.initStereoVision("testKalibracji.yml", 1, 2);
	while (waitKey(10) == -1)
	{
		stereoVision.grabFrames();
		stereoVision.undistortRectifyFrames(stereoVision.leftFrame, stereoVision.rightFrame);
		stereoVision.drawParallerLines(stereoVision.leftTransformedFrame);
		stereoVision.drawParallerLines(stereoVision.rightTransformedFrame);
		imshow("leftCam", stereoVision.leftTransformedFrame);
		imshow("rightCam", stereoVision.rightTransformedFrame);
	}

	return 1;
}