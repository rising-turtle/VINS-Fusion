/*
	July 23, 2019, He Zhang, hzhang8@vcu.edu 

	Estimator to handle depth data 


*/


#pragma once 
#include "estimator.h"


class EstimatorDpt : public Estimator
{
public:
	EstimatorDpt();
	~EstimatorDpt();

	void inputImageDpt(double t, const cv::Mat &_img, const cv::Mat &dpt = cv::Mat());

	float mbf; // base line * fx 

};