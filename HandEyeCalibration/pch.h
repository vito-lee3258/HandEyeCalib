#ifndef PCH_H
#define PCH_H
#define _CRT_SECURE_NO_WARNINGS
#define FENGZHUANGCPP_API __declspec(dllexport) //导出

#include <stdio.h>

extern "C" {
	struct Point1
	{
		int x;
		int y;
	};

	struct CameraInstrinsic
	{
		double Fx;
		double Fy;
		double Cx;
		double Cy;

		double K1;
		double K2;
		double P1;
		double P2;
		double K3;
	};

	struct GeneralHandEyeResult
	{
		double matrix[16];

		double corner_in_base_xyz[3];

		double RMS;

		double HandEyeMatrix[7];// x, y, z, qx, qy, qz, qw.
	};

	struct CornersPoints
	{
		double corner_point_0[2];
		double corner_point_1[2];
		double corner_point_57[2];
		double corner_point_63[2];
	};

	__declspec(dllexport) int __stdcall Add(int a, int b)
	{
		return a + b;
	}

	__declspec(dllexport) void __cdecl PrintMessage(const char* message)
	{
		printf("%s\n", message);
	}

	__declspec(dllexport) void __stdcall MovePoint(Point1* p, int dx, int dy)
	{
		p->x += dx;
		p->y += dy;
	}

	// OpenCV算法库。
	__declspec(dllexport) void __stdcall ConvertToGray(const char* inputPath, const char* outputPath);

	__declspec(dllexport) void __stdcall GetCameraIntrinsic(CameraInstrinsic* cameraIntrinsic);

	__declspec(dllexport) bool __stdcall CornerDetection(const char* inputPath, CornersPoints* corners);

	__declspec(dllexport) bool __cdecl Run(const char* imagePath, const char* pointCloudPath, const char* robotPosePath, 
									   CameraInstrinsic* cameraIntrinsic, GeneralHandEyeResult* CalibResult);

	// 梅卡手眼标定库。
	__declspec(dllexport) bool __stdcall InitCamera();

	__declspec(dllexport) bool __stdcall GetPatternImage();

	__declspec(dllexport) bool __stdcall AddRobotPose(double x, double y, double z, double rx, double ry, double rz);

	__declspec(dllexport) bool __stdcall Calibrate(GeneralHandEyeResult* CalibResult);
}

#endif //PCH_H