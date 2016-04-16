#pragma once
#include "opencv2\opencv.hpp"

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
	int undistortRectifyFrames(cv::Mat &leftFrame, cv::Mat &rightFrame);
	void showImage(cv::Mat image, bool waitForKey);
	void showImage(char* windowName, cv::Mat image, bool waitForKey);
	void drawParallerLines(cv::Mat &image);
	void calcDisparityMap();
	cv::Mat reproject();
	void initStereoMatcher();
	cv::Point findPoint(cv::Mat& img);

	int status;
	//String statusText; // przydatne?
	bool camsOpened;
	cv::VideoCapture leftCam, rightCam;
	cv::Ptr<cv::StereoBM> stereoMatcher;
	cv::Mat leftCameraMat, leftCameraDistorsion, rightCameraMat, rightCameraDistorsion;
	cv::Mat rotationMat, leftRectificationMat, leftProjectionMat,
					rightRectificationMat, rightProjectionMat;
	cv::Mat disparityToDepthMat;
	cv::Mat leftFrame, rightFrame;
	cv::Mat leftFilteredFrame, rightFilteredFrame;
	cv::Mat leftTransformedFrame, rightTransformedFrame;
	cv::Mat disparityMap;
	cv::Rect leftValidPixROI, rightValidPixROI;
	cv::Size imageSize;
};

