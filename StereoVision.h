#pragma once
#include "opencv2\opencv.hpp"
#include <math.h>

# define PI 3.14159265358979323846

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
	int filterFrames_BRIGHT(cv::Mat& left, cv::Mat& right);
	int undistortRectifyFrames(cv::Mat &leftFrame, cv::Mat &rightFrame);
	void showImage(cv::Mat image, bool waitForKey);
	void showImage(char* windowName, cv::Mat image, bool waitForKey);
	void drawParallerLines(cv::Mat &image);
	void calcDisparityMap();
	cv::Mat reproject();
	void initStereoMatcher();
	cv::Point2f findPoint(cv::Mat& img);
	cv::Mat triangulate(cv::Mat& leftImg, cv::Mat& rightImg);
	cv::Point3f coordinateTransform(cv::Point3f point, cv::Point3f trans, cv::Point3f rot);


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
	cv::Point3f coordnateTrans;
	cv::Point3f coordnateRot;
};

