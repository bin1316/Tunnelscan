// suibianshishi.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <iostream>
#include <boost/signals2.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
using namespace std;
union tofloat
{
	unsigned char data[4];
	struct bytes
	{
		unsigned char low_byte;
		unsigned char mlow_byte;
		unsigned char mhigh_byte;
		unsigned char high_byte;
	}fbyte;
	float value;
}myfloat;
void hello(){
	printf("helloworld\n");
}
void nihao()
{
	printf("nihao\n");
}
void dayin(string word)
{
	cout << word << endl;
}
int _tmain(int argc, _TCHAR* argv[])
{
	unsigned char a[4] = { 63, 154, 225, 72 };
	for (int i = 0; i < 4;i++)
	{
		myfloat.data[i] = a[3-i];
	}
	
	cout << myfloat.value << endl;
	return 0;
}

