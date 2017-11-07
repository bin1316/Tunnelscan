#ifndef KnearSearch_H_
#define KnearSearch_H_
#ifdef KnearSearch
#define KnearSearch extern "C" _declspec(dllimport) 
#else
#define KnearSearch extern "C" _declspec(dllexport) 
#endif // KnearSearch
#include <flann/flann.h>
flann::Index<flann::L2_Simple<float>> flannindex(flann::KDTreeSingleIndexParams(15));

KnearSearch void getflannindex(float* cloud, int size);
//KnearSearch void KNS(float* SP, float* cloud, int size, int k, int* indices, float* dist);
KnearSearch void KNS(float* SP, int k, int* indices, float* dist);
#endif
