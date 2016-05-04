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
	if (getPixelValue(point4D, 3, 1) == 0)
		point3D = Point3f(0, 0, 0);
	else
	{
		float w = getPixelValue(point4D, 4, 1);
		point3D.x = getPixelValue(point4D, 1, 1) / w;
		point3D.y = getPixelValue(point4D, 2, 1) / w;
		point3D.z = getPixelValue(point4D, 3, 1) / w;
	}

	return point3D;
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
	//namedWindow("rightCam");
	//namedWindow("depth");
	stereoVision.initStereoVision("testKalibracji.yml", 1, 2);

	if (!robotConnection.setupConnection(KAWASAKI_ADDRESS, KAWASAKI_PORT))
	{
		cout << "NIE NAWIAZANO POLACZENIA\n";
	}

	while ((waitKey(5) == -1))
	{
		stereoVision.grabFrames();
		stereoVision.undistortRectifyFrames(stereoVision.leftFrame, stereoVision.rightFrame);
		stereoVision.filterFrames_BRIGHT(stereoVision.leftTransformedFrame, stereoVision.rightTransformedFrame);
		imshow("leftCam", stereoVision.leftFrame);
		//imshow("rightCam", stereoVision.rightFrame);
		detectedPoint4D = stereoVision.triangulate(stereoVision.leftFilteredFrame, stereoVision.rightFilteredFrame);
		detectedPoint3D = calcPoint3D(detectedPoint4D);
		detectedPoint3D = stereoVision.coordinateTransform(detectedPoint3D, Point3f(-1000,-1000,700), Point3f(-135,0,0));	// punkt w odniesieniu do nowego ukl. wsp.
		if (detectedPoint3D != Point3f(0, 0, 0) && robotConnection.isConnected())
		{
			cout << "WYSYLAM...\n";
			std::string dataToSend = std::to_string(detectedPoint3D.x) + ";" +
				std::to_string(detectedPoint3D.y) + ";" +
				std::to_string(detectedPoint3D.z) + ";";
			if (robotConnection.sendData(dataToSend.c_str()) != 0)
			{
				cout << "WYSLANO:\n" << dataToSend.c_str() << endl;
			}
			//return 1;
		}
		//cout << getPixelValue(detectedPoint4D,1,1) << endl;
		cout << detectedPoint3D << endl << endl;
		//cout << plik.is_open() << endl;
		
		//saveToFile(plik, detectedPoint3D);
	}
	waitKey();
	//plik.close();
	/*
	CStereoCalibration calibrate;

	if (calibrate.openCameras(1, 2) != 1)
		return 0;
	calibrate.runCalibration();
	calibrate.saveSettings("testKalibracji.yml");
	waitKey();
	*/

	return 1;
}