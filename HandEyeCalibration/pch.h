#ifndef PCH_H
#define PCH_H
#define _CRT_SECURE_NO_WARNINGS
#define FENGZHUANGCPP_API __declspec(dllexport) //导出
//动态链接库
class FENGZHUANGCPP_API IInterface
{
public:
	static IInterface* CreateInterface();
	virtual void Init() = 0;
	virtual void Destroy() = 0;
	virtual char* GetName() = 0;

	virtual void LoadImage() = 0;
};
#endif //PCH_H