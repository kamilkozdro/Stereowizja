#include <iostream>
#include <opencv2\opencv.hpp>
#include "StereoCalibration.h"
#include "StereoVision.h"

using namespace cv;

int main()
{
	CStereoCalibration calibrate;
	CStereoVision stereoVision;


	calibrate.runCalibration();
	calibrate.saveSettings("testKalibracji.yml");

	return 1;
}