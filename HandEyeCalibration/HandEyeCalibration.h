#pragma once

#include "pch.h"
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

#include "area_scan_3d_camera/HandEyeCalibration.h"

using namespace cv;
using namespace std;

class Settings
{
public:
    enum Pattern { NOT_EXISTING, CHESSBOARD, CHARUCOBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
    enum InputType { INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST };

    int boardWidth = 4;
    int boardHeight = 5;
    //cv::Size boardSize(8, 11);
    Pattern calibrationPattern = Settings::ASYMMETRIC_CIRCLES_GRID;  // One of the Chessboard, ChArUco board, circles, or asymmetric circle pattern
    float squareSize = 18;            // The size of a square in your defined unit (point, millimeter,etc).
    float markerSize = 10;            // The size of a marker in your defined unit (point, millimeter,etc).
    string arucoDictName = "DICT_4X4_1000";        // The Name of ArUco dictionary which you use in ChArUco pattern
    string arucoDictFileName;    // The Name of file which contains ArUco dictionary for ChArUco pattern

    vector<string> imageList;

private:
    string patternToUse;
};

//bool InitCamera();
//
//bool GetPatternImage();
//
//bool AddRobotPose(double x, double y, double z, double rx, double ry, double rz);
//
void saveExtrinsicParameters(const std::string& ExtrinsicParameters);

mmind::eye::HandEyeCalibration::Transformation GetRobotPose(double x, double y, double z, double rx, double ry, double rz);

mmind::eye::HandEyeCalibration::Transformation EulerToQuad(int eulerType, double x, double y,
    double z, double R1, double R2, double R3);

int poseIndex = 1;

mmind::eye::Camera camera;
mmind::eye::HandEyeCalibration calibration;
mmind::eye::HandEyeCalibration::Transformation cameraToBase;

Mat cameraMatrix(3, 3, CV_32FC1);
Mat distortion(1, 5, CV_32FC1);

#define PI 3.14159265

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

void ImageEnhance(Mat image);

bool attitudeVector2Matrix(Mat m, Mat& R, Mat& T, bool isEuler);

Mat EulerToRotationMatrix(double pitch, double yaw, double roll);

