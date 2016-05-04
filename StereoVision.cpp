#include "StereoVision.h"

using namespace cv;

CStereoVision::CStereoVision()
{
	status = 0;
	camsOpened = false;
}

CStereoVision::~CStereoVision()
{

}

int CStereoVision::initStereoVision(char* path, int leftCamID = -1, int rightCamID = -1)
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

int CStereoVision::filterFrames_BRIGHT(Mat& left, Mat& right)
{
	int min1 = 0, min2 = 33, min3 = 70;
	int max1 = 255, max2 = 201, max3 = 255;
	Mat leftFrameHSV, rightFrameHSV;

	cvtColor(left, leftFrameHSV, CV_BGR2HSV);
	cvtColor(right, rightFrameHSV, CV_BGR2HSV);
	inRange(leftFrameHSV, Scalar(min1, min2, min3), Scalar(max1, max2, max3), leftFilteredFrame);
	inRange(rightFrameHSV, Scalar(min1, min2, min3), Scalar(max1, max2, max3), rightFilteredFrame);

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

Point2f CStereoVision::findPoint(Mat& img)
{
	float xMin = img.cols, xMax = 0, yMin = img.rows, yMax = 0;
	int counter = 0;
	uchar* pointer;
	for (int i = 0; i < img.rows; i++)
	{
		pointer = img.ptr(i);
		for (int j = 0; j < img.cols; j++)
		{
			if (pointer[j] == 255)
			{
				counter++;
				if (j < xMin)
					xMin = j;
				if (j > xMax)
					xMax = j;
				if (i < yMin)
					yMin = i;
				if (i > yMax)
					yMax = i;
			}
		}
	}
	//std::cout << "LICZBA PUNKTOW: " << counter << std::endl;
	if (counter == 1)	// jeden punkt odnaleziony
		return Point2f(xMax, yMax);
	else if (counter == 0)	// 0 punktow
		return NULL;
	else				// wiele punktow - blop
		return Point2f(xMin + (xMax - xMin) / 2, yMin + (yMax - yMin) / 2);
}

Mat CStereoVision::triangulate(Mat& leftImg, Mat& rightImg)
{
	std::vector<Point2f> leftPoint, rightPoint;
	Point2f left, right;
	Mat point4D = Mat(4, 1, CV_32F);

	left = findPoint(leftImg);
	right = findPoint(rightImg);
	if (left == right)	// czyli brak punktow / (0,0)
		point4D = Mat::zeros(4, 1, CV_32F);
	else
	{
		leftPoint.push_back(findPoint(leftImg));
		rightPoint.push_back(findPoint(rightImg));

		triangulatePoints(leftProjectionMat, rightProjectionMat,
			leftPoint, rightPoint, point4D);
	}

	return point4D;
}

Point3f  CStereoVision::coordinateTransform(Point3f point, Point3f trans, Point3f rot)
{
	Mat rotXMat = Mat::eye(4, 4, CV_32F);
	Mat rotYMat = Mat::eye(4, 4, CV_32F);
	Mat rotZMat = Mat::eye(4, 4, CV_32F);
	Mat transMat = Mat::eye(4, 4, CV_32F);
	Mat invRotXMat, invRotYMat, invRotZMat, invTransMat;
	Mat cameraPoint = Mat(point);

	if (point == Point3f(0, 0, 0))
		return Point3f(0, 0, 0);

	cameraPoint.resize(4);
	cameraPoint.at<float>(3, 0) = 1;

	transMat.at<float>(0, 3) = trans.x;
	transMat.at<float>(1, 3) = trans.y;
	transMat.at<float>(2, 3) = trans.z;

	rotXMat.at<float>(1, 1) = cos(rot.x * PI / 180);
	rotXMat.at<float>(1, 2) = -sin(rot.x * PI / 180);
	rotXMat.at<float>(2, 1) = sin(rot.x * PI / 180);
	rotXMat.at<float>(2, 2) = cos(rot.x * PI / 180);

	rotYMat.at<float>(0, 0) = cos(rot.y * PI / 180);
	rotYMat.at<float>(0, 2) = sin(rot.y * PI / 180);
	rotYMat.at<float>(2, 0) = -sin(rot.y * PI / 180);
	rotYMat.at<float>(2, 2) = cos(rot.y * PI / 180);

	rotZMat.at<float>(0, 0) = cos(rot.z * PI / 180);
	rotZMat.at<float>(0, 1) = -sin(rot.z * PI / 180);
	rotZMat.at<float>(1, 0) = sin(rot.z * PI / 180);
	rotZMat.at<float>(1, 1) = cos(rot.z * PI / 180);

	Mat result = transMat * rotXMat * rotYMat * rotZMat * cameraPoint;
	result.resize(3);

	return Point3f(result);
}