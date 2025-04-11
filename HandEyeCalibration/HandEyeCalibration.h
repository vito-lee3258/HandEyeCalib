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
    int boardHeight = 8;
    //cv::Size boardSize(8, 11);
    Pattern calibrationPattern = Settings::CHESSBOARD;  // One of the Chessboard, ChArUco board, circles, or asymmetric circle pattern
    float squareSize = 18;            // The size of a square in your defined unit (point, millimeter,etc).
    float markerSize = 10;            // The size of a marker in your defined unit (point, millimeter,etc).
    string arucoDictName = "DICT_4X4_1000";        // The Name of ArUco dictionary which you use in ChArUco pattern
    string arucoDictFileName;    // The Name of file which contains ArUco dictionary for ChArUco pattern

    vector<string> imageList;

private:
    string patternToUse;
};

Mat cameraMatrix(3, 3, CV_32FC1);
Mat distortion(1, 5, CV_32FC1);

const double PI = 3.14159265358979323846;  // 精确到18位小数

vector<Mat> myRobotPose;
vector<Mat> myCameraRotation;
vector<Mat> myCameraTransform;
vector<Mat> R_gripper2base;
vector<Mat> T_gripper2base;

bool ReadStringList(const string& filename, vector<string>& l);

bool GetRobotPose(const char* robotPosePath);

void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
    Settings::Pattern patternType);

bool GetCameraMatrixChessboard(const char* imagePath, cv::Mat CameraMatrix, cv::Mat CameraDistortion,
    vector<cv::Mat>& Rotation, vector<cv::Mat>& Transform, double& RMS);

bool GetCameraMatrixAruco(const char* imagePath, cv::Mat CameraMatrix, cv::Mat CameraDistortion,
    vector<cv::Mat>& Rotation, vector<cv::Mat>& Transform, double& RMS);

bool attitudeVector2Matrix(Mat m, Mat& R, Mat& T, bool isEuler);

Mat EulerToRotationMatrix(double pitch, double yaw, double roll);

