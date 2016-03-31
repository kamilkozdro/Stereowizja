#pragma once
#include "opencv2\opencv.hpp"

using namespace cv;

class CStereoVision
{
public:
	CStereoVision();
	~CStereoVision();

	int initStereoVision(char* path, int leftCamID, int rightCamID);
	int loadSettings(char* path);
	int openCameras(int leftCamID, int rightCamID);
	int closeCameras();
	int grabFrames();
	int filterFrames_RED(int BGmin,int BGmax, int Rmin);
	int undistortRectifyFrames(Mat leftFrame, Mat rightFrame);
	void showImage(Mat image, bool waitForKey);

	int status;
	//String statusText; // przydatne?
	bool camsOpened;
	VideoCapture leftCam, rightCam;
	Mat leftCameraMat, leftCameraDistorsion, rightCameraMat, rightCameraDistorsion;
	Mat rotationMat, leftRectificationMat, leftProjectionMat,
					rightRectificationMat, rightProjectionMat;
	Mat disparityToDepthMat;
	Mat leftFrame, rightFrame;
	Mat leftFilteredFrame, rightFilteredFrame;
	Mat leftTransformedFrame, rightTransformedFrame;
	Rect leftValidPixROI, rightValidPixROI;
	Size imageSize;
};

