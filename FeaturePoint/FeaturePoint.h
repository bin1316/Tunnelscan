#ifndef  FeaturePoint_H_
#define  FeaturePoint_H_
#ifdef FeaturePoint
#define FeaturePoint extern "C" _declspec(dllimport)
#else
#define  FeaturePoint extern "C" _declspec(dllexport)
#endif // FeaturePoint

FeaturePoint void DataExtract(double* dist, int* indices, int* isize);
#endif
