// HandEyeCalib.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include "../HandEyeCalibration/pch.h"

using namespace std;

#pragma comment(lib, "HandEyeCalibration.lib")

int main()
{
    IInterface* IF = IInterface::CreateInterface();

    cout << IF->GetName() << endl;

    IF->Init();

    IF->Destroy();

    return 0;
}
