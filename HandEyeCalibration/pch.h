#ifndef PCH_H
#define PCH_H
#define _CRT_SECURE_NO_WARNINGS
#define FENGZHUANGCPP_API __declspec(dllexport) //导出

#pragma pack(push, 1)//确保内存对齐一致
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
#pragma pack(pop)

//动态链接库
class FENGZHUANGCPP_API IInterface
{
public:
	static IInterface* CreateInterface();
	virtual void Run(const char* imagePath, const char* pointCloudPath, const char* robotPosePath, CameraInstrinsic* CameraMatrix) = 0;
};

extern "C" __declspec(dllexport) void RunHandEyeCalib(const char* imagePath, const char* pointCloudPath, const char* robotPosePath, CameraInstrinsic* CameraMatrix)
{
	IInterface* HandEyeCalib = IInterface::CreateInterface();

	HandEyeCalib->Run(imagePath, pointCloudPath, robotPosePath, CameraMatrix);
}
#endif //PCH_H