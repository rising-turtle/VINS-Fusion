/*

	Aug. 14, 2019, He Zhang, hzhang8@vcu.edu 

	use depth to compute mask to remove people above the floor plane 


*/

#pragma once

#include "feature_tracker.h"
#include <sensor_msgs/PointCloud2.h>

class MaskPeople; 

class FeatureTrackerMask : public FeatureTracker
{
public:
	FeatureTrackerMask(); 
	virtual ~FeatureTrackerMask(); 

	void setMaskP(cv::Mat& mask_p);
	virtual map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> trackImageDpt(double _cur_time, float mbf, const cv::Mat &_img, const cv::Mat &_img1 = cv::Mat());

	MaskPeople* mpMaskPeople; 
	sensor_msgs::PointCloud2 mask_pc; // masked point cloud 
};