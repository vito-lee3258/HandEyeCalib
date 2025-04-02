// HandEyeCalib.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include "../HandEyeCalibration/pch.h"

using namespace std;

#pragma comment(lib, "HandEyeCalibration.lib")

int main()
{
    const char* imagePath = "E:\\Projects\\HandEyeCalib\\Version3.0\\HandEyeCalib\\x64\\Debug\\Data\\Image.txt";
    const char* pointCloudPath = ".\\Data\\";
    const char* robotPosePath = "E:\\Projects\\HandEyeCalib\\Version3.0\\HandEyeCalib\\x64\\Debug\\Data\\Pose.txt";

    CameraInstrinsic cameraMatrix;
    cameraMatrix.Fx = 0;
    RunHandEyeCalib(imagePath, pointCloudPath, robotPosePath, &cameraMatrix);

    return 0;
}
