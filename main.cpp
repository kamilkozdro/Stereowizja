#include <iostream>
#include <opencv2\opencv.hpp>
#include "StereoCalibration.h"
#include "StereoVision.h"
#include "TCPConnection.h"

#define KAWASAKI_ADDRESS "192.168.0.3"
#define KAWASAKI_PORT "49152"

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
	CTCPConnection robotConnection;
	namedWindow("leftCam");
	//namedWindow("rightCam");
	namedWindow("depth");
	/*
	stereoVision.initStereoVision("testKalibracji.yml", 1, 2);
	//while (waitKey(5) == -1)
	{
		//stereoVision.leftCam.set(CV_CAP_PROP_EXPOSURE, stereoVision.rightCam.get(CV_CAP_PROP_EXPOSURE));
		//stereoVision.leftCam.set(CV_CAP_PROP_GAIN, stereoVision.rightCam.get(CV_CAP_PROP_GAIN));
		//stereoVision.grabFrames();
		stereoVision.leftFrame = imread("C:/Users/Hp/Desktop/Air/praca mgr/kamera_kalib/stereowizja/obrazy/lewa_punkt.png");
		stereoVision.rightFrame = imread("C:/Users/Hp/Desktop/Air/praca mgr/kamera_kalib/stereowizja/obrazy/prawa_punkt.png");
		stereoVision.undistortRectifyFrames(stereoVision.leftFrame, stereoVision.rightFrame);
		//stereoVision.drawParallerLines(stereoVision.leftTransformedFrame);
		//stereoVision.drawParallerLines(stereoVision.rightTransformedFrame);
		imshow("leftCam", stereoVision.leftTransformedFrame);
		//imshow("rightCam", stereoVision.rightTransformedFrame);
		stereoVision.calcDisparityMap();
		//std::cout << stereoVision.disparityMap.size() << endl;
		//depth = stereoVision.reproject();
		imshow("depth", stereoVision.disparityMap);
		waitKey();
	}
	*/

	if (!robotConnection.setupConnection("127.0.0.1", "27015"))
		return 0;
	robotConnection.sendData("Heheszki C:\n");
	robotConnection.closeConnection();
	return 1;
}