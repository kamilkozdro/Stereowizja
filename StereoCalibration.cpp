#include "StereoCalibration.h"

using namespace cv;

CStereoCalibration::CStereoCalibration()
{
	chessboardSize.width = 9;
	chessboardSize.height = 6;
	squareSize = 25;
	timer = 0;
}

CStereoCalibration::~CStereoCalibration()
{
}

vector<vector<Point3f>> CStereoCalibration::calcObjectPoints(int imagesNumber)
{
	vector<vector<Point3f>> objectPoints;

	objectPoints.resize(imagesNumber);
	// zalozenie: wszystkie pola w osi Z = 0; rownolegle do obiektywu
	for (int i = 0; i < imagesNumber; i++)
	{
		for (int j = 0; j < chessboardSize.height; j++)
		{ 
			for (int k = 0; k < chessboardSize.width; k++)
				objectPoints[i].push_back(Point3f(float(k*squareSize), float(j*squareSize), 0));
		}		
	}

	return objectPoints;
}

int CStereoCalibration::getCalibImagePoints(vector<Mat>& frames, bool showFrames = false, int delay = 0)
{
	bool leftFound, rightFound;
	vector<Point2f>leftImagePointsBuffer, rightImagePointsBuffer;

	for (int i = 0; i < frames.size(); i++)
	{
		leftFound = findChessboardCorners(frames[i], chessboardSize, leftImagePointsBuffer,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		drawChessboardCorners(frames[i], chessboardSize, leftImagePointsBuffer, leftFound);
		if (showFrames)
			showImage("leftCam", frames[i], false);
		rightFound = findChessboardCorners(frames[++i], chessboardSize, rightImagePointsBuffer,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		drawChessboardCorners(frames[i], chessboardSize, rightImagePointsBuffer, rightFound);
		if (showFrames)
			showImage("rightCam", frames[i], false);
	
		if (frames[i - 1].size() != frames[i].size())
				return 0; // ROZNE ROZMIARY OBRAZOW!!!
		if (rightFound && leftFound)
		{
			if (timerElapsed() >= delay || timer == 0)
			{
				leftImagePoints.push_back(leftImagePointsBuffer);
				leftCalibFrames.push_back(frames[i - 1]);
				rightImagePoints.push_back(rightImagePointsBuffer);
				rightCalibFrames.push_back(frames[i]);
				std::cout << "FRAMES: " << leftCalibFrames.size() << endl;
				timerStart();
			}
		}
		leftImagePointsBuffer.clear();
		rightImagePointsBuffer.clear();
	}

	return 1;
}

void CStereoCalibration::loadFrames(vector<Mat>& frames, int flag = IMREAD_GRAYSCALE)
{
	frames.push_back(imread("C:/Users/Hp/Desktop/Air/praca mgr/kamera_kalib/stereo_kalib/obrazy/lewa_1.png", flag));
	frames.push_back(imread("C:/Users/Hp/Desktop/Air/praca mgr/kamera_kalib/stereo_kalib/obrazy/prawa_1.png", flag));
	frames.push_back(imread("C:/Users/Hp/Desktop/Air/praca mgr/kamera_kalib/stereo_kalib/obrazy/lewa_2.png", flag));
	frames.push_back(imread("C:/Users/Hp/Desktop/Air/praca mgr/kamera_kalib/stereo_kalib/obrazy/prawa_2.png", flag));
	frames.push_back(imread("C:/Users/Hp/Desktop/Air/praca mgr/kamera_kalib/stereo_kalib/obrazy/lewa_3.png", flag));
	frames.push_back(imread("C:/Users/Hp/Desktop/Air/praca mgr/kamera_kalib/stereo_kalib/obrazy/prawa_3.png", flag));
}

int CStereoCalibration::openCameras(int leftCamID, int rightCamID)
{
	leftCam.open(leftCamID);
	if (!leftCam.isOpened())
		return 0;
	namedWindow("leftCam");
	rightCam.open(rightCamID);
	if (!rightCam.isOpened())
		return 0;
	namedWindow("rightCam");

	camsOpened = true;
	return 1;
}

int CStereoCalibration::closeCameras()
{
	if (leftCam.isOpened())
		leftCam.release();
	if (rightCam.isOpened())
		rightCam.release();
	camsOpened = false;

	return 1;
}

void CStereoCalibration::saveSettings(char* path)
{
	FileStorage fileStream;
	time_t actualTime;

	fileStream.open(path, FileStorage::WRITE);
	time(&actualTime);
	fileStream << "calibrationDate" << asctime(localtime(&actualTime));
	fileStream << "leftCameraMat" << leftCameraMat;
	fileStream << "leftCameraDistorsion" << leftCameraDistorsion;
	fileStream << "rightCameraMat" << rightCameraMat;
	fileStream << "rightCameraDistorsion" << rightCameraDistorsion;
	fileStream << "rotationMat" << rotationMat;
	fileStream << "translationMat" << translationMat;
	fileStream << "leftRectificationMat" << leftRectificationMat;
	fileStream << "leftProjectionMat" << leftProjectionMat;
	fileStream << "rightRectificationMat" << rightRectificationMat;
	fileStream << "rightProjectionMat" << rightProjectionMat;
	fileStream << "disparity2DepthMat" << disparityToDepthMat;
	fileStream << "leftValidPixROI" << leftValidPixROI;
	fileStream << "rightValidPixROI" << rightValidPixROI;
	fileStream << "imageSize" << imageSize;
	fileStream.release();
}

void CStereoCalibration::showImage(Mat image, bool waitForKey = false)
{
	namedWindow("window");
	imshow("window", image);
	if (waitForKey)
		waitKey();
	destroyWindow("window");
}

void CStereoCalibration::showImage(char* windowName, Mat image, bool waitForKey = false)
{
	imshow(windowName, image);
	if (waitForKey)
		waitKey();
}

int CStereoCalibration::runCalibration()
{

	if (camsOpened)
	{
		vector<Mat> frames(2);
		int samplesRequired = 30;

		while (leftCalibFrames.size() < samplesRequired)
		{
			waitKey(1);	// PO CO?
			leftCam >> frames[0];
			rightCam >> frames[1];
			getCalibImagePoints(frames, true, 2);
		}
	}
	else
	{
		vector<Mat> frames;

		loadFrames(frames);
		getCalibImagePoints(frames);
	}

	imageSize = leftCalibFrames[0].size();
	vector<vector<Point3f>> objectPoints;
	objectPoints = calcObjectPoints(leftCalibFrames.size());
	leftCameraMat = initCameraMatrix2D(objectPoints, leftImagePoints, imageSize, 0);
	rightCameraMat = initCameraMatrix2D(objectPoints, rightImagePoints, imageSize, 0);

	error_rms = stereoCalibrate(objectPoints, leftImagePoints, rightImagePoints,
		leftCameraMat, leftCameraDistorsion, rightCameraMat, rightCameraDistorsion,
		imageSize, rotationMat, translationMat, essentialMat, fundamentalMat,
		CALIB_ZERO_TANGENT_DIST +
		CALIB_SAME_FOCAL_LENGTH,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
	stereoRectify(leftCameraMat, leftCameraDistorsion, rightCameraMat, rightCameraDistorsion,
		imageSize, rotationMat, translationMat,
		leftRectificationMat, rightRectificationMat, 
		leftProjectionMat, rightProjectionMat, disparityToDepthMat, 0, -1, imageSize, &leftValidPixROI, &rightValidPixROI);

	std::cout << "ZAKONCZONO KALIBRACJE!\nBLAD RMS = "<< error_rms << endl;

	return 1;
}