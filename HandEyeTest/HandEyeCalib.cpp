// HandEyeCalib.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include "../HandEyeCalibration/pch.h"

using namespace std;

#pragma comment(lib, "HandEyeCalibration.lib")

int main()
{
    const char* imagePath = "./Data/Image.txt";
    const char* pointCloudPath = ".\\Data\\";
    const char* robotPosePath = "./Data/Pose.txt";

    CameraInstrinsic cameraMatrix;
    cameraMatrix.Fx = 1575.8350830078125;
    cameraMatrix.Fy = 1575.98876953125;
    cameraMatrix.Cx = 736.35321044921875;
    cameraMatrix.Cy = 582.20849609375;

    cameraMatrix.K1 = -0.0970213041;
    cameraMatrix.K2 = 0.1625302583;
    cameraMatrix.P1 = -0.0611124262;
    cameraMatrix.P2 = -0.0001548736;
    cameraMatrix.K3 = -0.000862356;

    CornerDetection("./Data/0.png");

    GeneralHandEyeResult CalibResult;
    Run(imagePath, pointCloudPath, robotPosePath, &CalibResult);

    return 0;
}
