/*
	July 23, 2019, He Zhang, hzhang8@vcu.edu 

	Estimator to handle depth data 


*/


#include "estimator_dpt.h"
#include "../utility/visualization.h"

extern void handle_dpt(const cv::Mat& dpt, cv::Mat& dpt_out);
extern void handle_rgb(const cv::Mat& rgb, cv::Mat& rgb_out);

EstimatorDpt::EstimatorDpt(){

	// TODO: parameterize this 
	mbf = 0.1 * 444.277; // 532.388672; // 444.277; //featureTracker.m_camera[0]->fx;
}

EstimatorDpt::~EstimatorDpt(){}

void EstimatorDpt::inputImageDpt(double t, const cv::Mat &_img, const cv::Mat &dpt)
{
	inputImageCnt++;
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    TicToc featureTrackerTime;
    cv::Mat dpt_align; 
    cv::Mat img_undist; 

    if(dpt.empty()){
        featureFrame = featureTracker.trackImage(t, _img);
    }
    else{
    	handle_rgb(_img, img_undist);
    	handle_dpt(dpt, dpt_align);
        // img_undist = _img.clone(); 
        // dpt_align = dpt.clone();
        featureFrame = featureTracker.trackImageDpt(t, mbf, img_undist, dpt_align);
    }
    //printf("featureTracker time: %f\n", featureTrackerTime.toc());

    if (SHOW_TRACK)
    {
        cv::Mat imgTrack = featureTracker.getTrackImage();
        pubTrackImage(imgTrack, t);
        if(!featureTracker.mask_p.empty())
            pubMaskImage(featureTracker.mask_p, t);
    }
    
    if(MULTIPLE_THREAD)  
    {     
        if(inputImageCnt % 2 == 0)
        {
            mBuf.lock();
            featureBuf.push(make_pair(t, featureFrame));
            mBuf.unlock();
        }
    }
    else
    {
        mBuf.lock();
        featureBuf.push(make_pair(t, featureFrame));
        mBuf.unlock();
        TicToc processTime;
        processMeasurements();
        printf("process time: %f\n", processTime.toc());
    }
}
