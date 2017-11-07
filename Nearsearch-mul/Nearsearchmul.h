/*
 * Nearsearch-mul.h
 *
 *  Created on: 2017Äê5ÔÂ3ÈÕ
 *      Author: BIN
 */

#ifndef NEARSEARCHMUL_H_
#define NEARSEARCHMUL_H_
#ifdef NearSearchmul
#define NearSearchmul extern "C" _declspec(dllimport) 
#else
#define NearSearchmul extern "C" _declspec(dllexport) 
#endif // NearSearchmul
#include <flann/flann.h>
flann::Index<flann::L2_Simple<float>/* */> *flannindex;
typedef struct{
	float x,y,z;
}coordinate;
NearSearchmul void getflannindexmul(float* cloud, int size);
NearSearchmul int KNSmul(coordinate* SP, int conum, int k, int* indices, float* dist, float threshold);
NearSearchmul void deleteptrmul();




#endif /* NEARSEARCH_MUL_H_ */
