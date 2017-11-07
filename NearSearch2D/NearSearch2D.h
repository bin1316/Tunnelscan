#ifndef NearSearch2D_H_
#define NearSearch2D_H_
#ifdef NearSearch2D
#define NearSearch2D extern "C" _declspec(dllimport) 
#else
#define NearSearch2D extern "C" _declspec(dllexport) 
#endif // NearSearch2D
#include <flann/flann.h>
flann::Index<flann::L2_Simple<float>>* flannindex2d1;
flann::Index<flann::L2_Simple<float>>* flannindex2d2;
float* cloud2d1;
float* cloud2d2;
NearSearch2D void getflannindex2d(float* cloud1, int size1, float* cloud2, int size2);
NearSearch2D int KNS2d(float* SP, int k, int* indices1, float* dist1, int* indices2, float* dist2);
NearSearch2D void deleteptr();

#endif
