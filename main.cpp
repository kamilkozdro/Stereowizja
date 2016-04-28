#include <iostream>
#include <fstream>
#include <string>
#include <opencv2\opencv.hpp>
#include "StereoCalibration.h"
#include "StereoVision.h"
#include "TCPConnection.h"

#define KAWASAKI_ADDRESS "11.12.1.30"
#define KAWASAKI_PORT "9001"

using namespace cv;

float getPixelValue(Mat& img, int x, int y)
{
	/*
	float b = img.data[img.channels()*(img.cols*y + x) + 0];
	return b;
	*/
	float* ptr = img.ptr<float>(x-1);
	return ptr[y-1];
}

void saveToFile(ofstream& file, Point3f& point)
{
	if (file.is_open())
	{
		file << point.x << ";"
			<< point.y << ";"
			<< point.z << "\n";
	}
}

Point3f calcPoint3D(Mat& point4D)
{
	Point3f point3D;
	float w = getPixelValue(point4D, 4, 1);
	point3D.x = getPixelValue(point4D, 1, 1) / w;
	point3D.y = getPixelValue(point4D, 2, 1) / w;
	point3D.z = getPixelValue(point4D, 3, 1) / w;
}

int main()
{
	//ofstream plik;
	//plik.open("C:/Users/Hp/Desktop/Air/praca mgr/kamera_kalib/stereowizja/stereowizja/pomiary.txt", std::ios::out);
	CStereoVision stereoVision;
	Mat detectedPoint4D;
	Point3f detectedPoint3D;
	CTCPConnection robotConnection;
	namedWindow("leftCam");
	namedWindow("rightCam");
	//namedWindow("depth");
	
	stereoVision.initStereoVision("testKalibracji.yml", 1, 2);
	while (waitKey(5) == -1)
	{
		//stereoVision.leftCam.set(CV_CAP_PROP_EXPOSURE, stereoVision.rightCam.get(CV_CAP_PROP_EXPOSURE));
		//stereoVision.leftCam.set(CV_CAP_PROP_GAIN, stereoVision.rightCam.get(CV_CAP_PROP_GAIN));
		stereoVision.grabFrames();
		//stereoVision.leftFrame = imread("C:/Users/Hp/Desktop/Air/praca mgr/kamera_kalib/stereowizja/obrazy/lewa_punkt.png");
		//stereoVision.rightFrame = imread("C:/Users/Hp/Desktop/Air/praca mgr/kamera_kalib/stereowizja/obrazy/prawa_punkt.png");
		stereoVision.undistortRectifyFrames(stereoVision.leftFrame, stereoVision.rightFrame);
		stereoVision.filterFrames_BRIGHT(stereoVision.leftTransformedFrame, stereoVision.rightTransformedFrame);
		//stereoVision.drawParallerLines(stereoVision.leftTransformedFrame);
		//stereoVision.drawParallerLines(stereoVision.rightTransformedFrame);
		imshow("leftCam", stereoVision.leftFrame);
		//imshow("rightCam", stereoVision.rightFrame);
		detectedPoint4D = stereoVision.triangulate(stereoVision.leftFilteredFrame, stereoVision.rightFilteredFrame);
		detectedPoint3D = calcPoint3D(detectedPoint4D);
		//detectedPoint3D = stereoVision.coordinateTransform();	// punkt w odniesieniu do nowego ukl. wsp.
		//cout << getPixelValue(detectedPoint4D,1,1) << endl;
		//cout << detectedPoint4D << endl;
		//cout << plik.is_open() << endl;
		
		//saveToFile(plik, detectedPoint3D);
		
		//stereoVision.calcDisparityMap();
		//std::cout << stereoVision.disparityMap.size() << endl;
		//depth = stereoVision.reproject();
		//imshow("depth", stereoVision.disparityMap);
		//waitKey();
	}
	//plik.close();
	/*
	CStereoCalibration calibrate;

	if (calibrate.openCameras(1, 2) != 1)
		return 0;
	calibrate.runCalibration();
	calibrate.saveSettings("testKalibracji.yml");
	waitKey();
	*/
	/*
	stereoVision.initStereoVision("testKalibracji.yml", 1, 2);
	stereoVision.leftFrame = imread("C:/Users/Hp/Desktop/Air/praca mgr/kamera_kalib/stereowizja/obrazy/test_L.jpg");
	stereoVision.rightFrame = imread("C:/Users/Hp/Desktop/Air/praca mgr/kamera_kalib/stereowizja/obrazy/test_P.jpg");
	stereoVision.filterFrames_BRIGHT();

	cout << stereoVision.triangulate(stereoVision.leftFilteredFrame, stereoVision.rightFilteredFrame) << endl;
	waitKey();
	*/
	/*
	waitKey();
	if (!robotConnection.setupConnection(KAWASAKI_ADDRESS, KAWASAKI_PORT))
	{
		cout << "NIE NAWIAZANO POLACZENIA\n";
	}
	else
	{
		cout << "WYSYLAM...\n";
		std::string dataToSend = std::to_string(detectedPoint3D.x) + ";" +
			std::to_string(detectedPoint3D.y) + ";" +
			std::to_string(detectedPoint3D.z);
		if (robotConnection.sendData(dataToSend.c_str()) != 0)
		{
			
		}
		robotConnection.closeConnection();
	}
	
	waitKey();
	*/
	return 1;
}