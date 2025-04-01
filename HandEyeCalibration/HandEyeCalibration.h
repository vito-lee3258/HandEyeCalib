#pragma once

#include "pch.h"
#include<iostream>
#include<opencv2/highgui/highgui.hpp>

using namespace cv;

class HandEyeCalibration :public IInterface
{
public:
	HandEyeCalibration();
	virtual void Init();
	virtual void Destroy();
	virtual void LoadImage();
	virtual char* GetName();
private:
	char name[1024];
};