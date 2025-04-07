#pragma once

#include "pch.h"
#include <plog/Log.h>
#include <plog/Initializers/RollingFileInitializer.h>
#include <plog/Formatters/TxtFormatter.h>
#include <plog/Appenders/ColorConsoleAppender.h>
#include <fstream>
#include<iostream>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/objdetect/charuco_detector.hpp"

using namespace cv;
using namespace std;

class Settings
{
public:
    enum Pattern { NOT_EXISTING, CHESSBOARD, CHARUCOBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
    enum InputType { INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST };

    int boardWidth = 8;
    int boardHeight = 11;
    //cv::Size boardSize(8, 11);
    Pattern calibrationPattern = Settings::CHARUCOBOARD;  // One of the Chessboard, ChArUco board, circles, or asymmetric circle pattern
    float squareSize = 20;            // The size of a square in your defined unit (point, millimeter,etc).
    float markerSize = 10;            // The size of a marker in your defined unit (point, millimeter,etc).
    string arucoDictName = "DICT_4X4_50";        // The Name of ArUco dictionary which you use in ChArUco pattern
    string arucoDictFileName;    // The Name of file which contains ArUco dictionary for ChArUco pattern
    int nrFrames;                // The number of frames to use from the input for calibration
    float aspectRatio;           // The aspect ratio
    int delay;                   // In case of a video input
    bool calibZeroTangentDist;   // Assume zero tangential distortion
    bool calibFixPrincipalPoint; // Fix the principal point at the center
    bool flipVertical;           // Flip the captured images around the horizontal axis
    string outputFileName;       // The name of the file where to write
    bool showUndistorted;        // Show undistorted images after calibration
    string input;                // The input ->
    bool useFisheye;             // use fisheye camera model for calibration
    bool fixK1;                  // fix K1 distortion coefficient
    bool fixK2;                  // fix K2 distortion coefficient
    bool fixK3;                  // fix K3 distortion coefficient
    bool fixK4;                  // fix K4 distortion coefficient
    bool fixK5;                  // fix K5 distortion coefficient

    vector<string> imageList;
    size_t atImageList;
    InputType inputType;
    bool goodInput;
    int flag;

private:
    string patternToUse;
};

class HandEyeCalibration : public IInterface
{
public:
	// 构造函数。
	HandEyeCalibration();

	// 析构函数。
	~HandEyeCalibration();

	// 接口函数。
	virtual void Run(const char* imagePath, const char* pointCloudPath, const char* robotPosePath, CameraInstrinsic* CameraMatrix);

	static void GetTime(string timeStamp)
	{
		time_t now = time(nullptr);  // 获取自1970年以来的秒数（UTC时间戳）
		tm* local = localtime(&now); // 转换为本地时间结构体

		// 自定义格式化输出
		char time[80];
		strftime(time, 80, "%Y-%m-%d %H:%M:%S", local);
		std::cout << "格式化时间: " << time << std::endl;

		timeStamp = time;
	}

    bool ReadStringList(const string& filename, vector<string>& l);
	bool GetRobotPose(const char* robotPosePath);
	bool GetCameraMatrix(const char* imagePath, cv::Mat CameraMatrix, cv::Mat CameraDistortion, vector<cv::Mat> &Rotation, vector<cv::Mat> &Transform);
    bool attitudeVector2Matrix(Mat m, Mat &R, Mat &T, bool isEuler);
    Mat EulerToRotationMatrix(double pitch, double yaw, double roll);
private:
    const double PI = 3.14159265358979323846;  // 精确到18位小数

	vector<Mat> myRobotPose;
    vector<Mat> myCameraRotation;
    vector<Mat> myCameraTransform;
    vector<Mat> R_gripper2base;
    vector<Mat> T_gripper2base;

};

