#include "StereoVision.h"


CStereoVision::CStereoVision()
{
	status = 0;
	camsOpened = false;
}

CStereoVision::~CStereoVision()
{

}

int CStereoVision::initStereoVision(char* path, int leftCamID, int rightCamID)
{
	if (loadSettings(path) != 1)
		return 0;
	if (openCameras(leftCamID, rightCamID) != 1)
		return 0;
	
	status = 1;
	return 1;

}

int CStereoVision::loadSettings(char* path)
{
	FileStorage fileStream;
	fileStream.open(path, FileStorage::READ);
	if (!fileStream.isOpened())
		return 0;

	fileStream["leftCameraMat"] >> leftCameraMat;
	fileStream["leftCameraDistorsion"] >> leftCameraDistorsion;
	fileStream["rightCameraMat"] >> rightCameraMat;
	fileStream["rightCameraDistorsion"] >> rightCameraDistorsion;
	fileStream["rotationMat"] >> rotationMat;
	fileStream["leftRectificationMat"] >> leftRectificationMat;
	fileStream["leftProjectionMat"] >> leftProjectionMat;
	fileStream["rightRectificationMat"] >> rightRectificationMat;
	fileStream["rightProjectionMat"] >> rightProjectionMat;
	fileStream["disparity2DepthMat"] >> disparityToDepthMat;
	fileStream["leftValidPixROI"] >> leftValidPixROI;
	fileStream["rightValidPixROI"] >> rightValidPixROI;
	fileStream["imageSize"] >> imageSize;
	fileStream.release();

	return 1;
}

int CStereoVision::openCameras(int leftCamID, int rightCamID)
{
	leftCam.open(leftCamID);
	if (!leftCam.isOpened())
		return 0;
	rightCam.open(rightCamID);
	if (!rightCam.isOpened())
		return 0;
	
	camsOpened = true;
	return 1;
}

int CStereoVision::closeCameras()
{
	if (leftCam.isOpened())
		leftCam.release();
	if (rightCam.isOpened())
		rightCam.release();
	camsOpened = false;

	return 1;
}

int CStereoVision::grabFrames()
{
	if (!camsOpened)
		return 0;
	leftCam >> leftFrame;
	rightCam >> rightFrame;

	return 1;
}

int CStereoVision::filterFrames_RED(int BGmin = 0, int BGmax = 10, int Rmin = 200)
{
	if (!status)
		return 0;
	inRange(leftFrame, Scalar(BGmin, BGmin, Rmin), Scalar(BGmax, BGmax, 255), leftFilteredFrame);
	inRange(rightFrame, Scalar(BGmin, BGmin, Rmin), Scalar(BGmax, BGmax, 255), rightFilteredFrame);

	return 1;
}

int CStereoVision::undistortRectifyFrames(Mat &leftImage, Mat &rightImage)
{
	Mat leftMapX, leftMapY, rightMapX, rightMapY;
	// undystorsje przerzucic do CStereoCalib?
	initUndistortRectifyMap(leftCameraMat, leftCameraDistorsion, leftRectificationMat, leftProjectionMat, imageSize, CV_32FC1, leftMapX, leftMapY);
	initUndistortRectifyMap(rightCameraMat, rightCameraDistorsion, rightRectificationMat, rightProjectionMat, imageSize, CV_32FC1, rightMapX, rightMapY);

	remap(leftImage, leftTransformedFrame, leftMapX, leftMapY, INTER_LINEAR);
	remap(rightImage, rightTransformedFrame, rightMapX, rightMapY, INTER_LINEAR);

	return 1;
}

void CStereoVision::showImage(Mat image, bool waitForKey)
{
	namedWindow("window");
	imshow("window", image);
	if (waitForKey)
		waitKey();
	destroyWindow("window");
}

void CStereoVision::showImage(char* windowName, Mat image, bool waitForKey = 0)
{
	imshow(windowName, image);
	if (waitForKey)
		waitKey();
}

void CStereoVision::drawParallerLines(Mat & image)
{
	Size imageSize = image.size();
	
	for (int i = 0; i < imageSize.height; i+=64)
	{
		line(image, Point(0, i), Point(imageSize.width, i), Scalar(0, 255, 0),1);
	}
}

void CStereoVision::calcDisparityMap()
{
	Mat disparityMap;
	Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 16, 3, 0, 0, 1, 63, 10, 100, 32, StereoSGBM::MODE_SGBM);
	sgbm->compute(leftTransformedFrame, rightTransformedFrame, disparityMap);
}

Mat CStereoVision::reproject()
{
	Mat xyz;
	reprojectImageTo3D(disparityMap, xyz, disparityToDepthMat, false);

	return xyz;
}
