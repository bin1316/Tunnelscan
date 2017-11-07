// test.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <NearSearch2D.h>
#include <FeaturePoint.h>
#include <iostream>
using namespace std;
int _tmain(int argc, _TCHAR* argv[])
{
	float a[24] = { 1, 2, 3, 4, 5, 6, 8, 0, 1, 2, 3, 8, 9, 10, 6, 8, 1, 2, 3, 4, 5, 6, 4, 0 };
	float p[] = { 1, 1, 2 };
	int size = 0;
	double b[16][24];
	for (int i = 0; i < 16; i++)
		for (int j = 0; j < 24; j++)
			b[i][j] = a[j];
	int indice[384];
	DataExtract(b[0], indice, &size);
	for (size_t i = 0; i < size; i++)
	{
		cout << *(b[0]+*(indice+i)) << endl;
	}
	//DataExtract(a, 9, indice, &size);
	return 0;
}

