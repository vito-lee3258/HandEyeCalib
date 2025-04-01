#include "pch.h"
#include<iostream>
class FHello :public IInterface
{
public:
	FHello();
	virtual void Init();
	virtual void Destroy();
	virtual char* GetName();
private:
	char name[1024];
};
FHello::FHello()
{
	memset(name, 0, 1024);
	strcpy(name, "hello");
}
void FHello::Init()
{
	printf("FHello::Init\n");
}
void FHello::Destroy()
{
	printf("FHello::Destroy\n");
}
char* FHello::GetName()
{
	return name;
}
IInterface* IInterface::CreateInterface()
{
	return new FHello();
}