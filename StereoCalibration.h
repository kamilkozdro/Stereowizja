#pragma once
#include "opencv2\opencv.hpp"
#include <time.h>
#include <iostream>

using namespace cv;
using namespace std;

class CStereoCalibration
{
public:
	CStereoCalibration();
	~CStereoCalibration();

	vector<vector<Point3f>> calcObjectPoints(int imagesNumber);
	int getCalibImagePoints(vector<Mat>& frames, bool showFrames, int delay);
	//int getCalibFrames(VideoCapture& cap, vector<Mat>& outputArrayMat, size_t numberOfFrames, vector<vector<Point2f>>& outputImagePoints);
	void loadFrames(vector<Mat>& frames, int flag);
	int openCameras(int leftCamID, int rightCamID);
	int closeCameras();
	void saveSettings(char* path);
	void showImage(Mat image, bool waitForKey);
	void showImage(char* windowName, Mat image, bool waitForKey);
	int runCalibration();

	inline void timerStart() { timer = (double)getTickCount(); };
	inline double timerElapsed() { return ((double)getTickCount() - timer) / getTickFrequency(); };

	bool camsOpened;
	double timer;
	VideoCapture leftCam, rightCam;
	vector<Mat> leftCalibFrames, rightCalibFrames;
	vector<vector<Point2f>> leftImagePoints, rightImagePoints;
	Mat leftCameraMat, leftCameraDistorsion, rightCameraMat, rightCameraDistorsion;
	Mat rotationMat, translationMat, essentialMat, fundamentalMat,
		leftRectificationMat, leftProjectionMat,
		rightRectificationMat, rightProjectionMat;
	Mat disparityToDepthMat;
	Mat leftFrame, rightFrame;
	Mat leftFilteredFrame, rightFilteredFrame;
	Mat leftTransformedFrame, rightTransformedFrame;
	Rect leftValidPixROI, rightValidPixROI;
	Size imageSize;
	Size chessboardSize;
	int squareSize;

};

