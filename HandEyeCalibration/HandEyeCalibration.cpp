#include "pch.h"
#include "HandEyeCalibration.h"

HandEyeCalibration::HandEyeCalibration()
{
	memset(name, 0, 1024);
	strcpy(name, "hello");
}

void HandEyeCalibration::Init()
{
	printf("FHello::Init\n");
}

void HandEyeCalibration::Destroy()
{
	printf("FHello::Destroy\n");
}

char* HandEyeCalibration::GetName()
{
	return name;
}

void HandEyeCalibration::LoadImage()
{
	cv::Mat img = cv::imread("1.png");

	imwrite("test.png", img);
}

IInterface* IInterface::CreateInterface()
{
	return new HandEyeCalibration();
}