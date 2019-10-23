/*
	July 23, 2019, He Zhang, hzhang8@vcu.edu 

	Estimator to handle depth data 


*/


#include "estimator_dpt.h"
#include "../utility/visualization.h"

extern void handle_dpt(const cv::Mat& dpt, cv::Mat& dpt_out);
extern void handle_rgb(const cv::Mat& rgb, cv::Mat& rgb_out);

EstimatorDpt::EstimatorDpt(){

	// need to set parameterize for this 
	mbf = -1.;  // 0.1 * 444.277; // 532.388672; // 444.277; //featurbeTracker.m_camera[0]->fx;
    mb_calibrated = false; 
    cout << "estimator_dpt.cpp: mbf = "<<mbf<<" need to set explicitly!"<<endl;
}

EstimatorDpt::~EstimatorDpt(){}

void EstimatorDpt::setParameter()
{
    Estimator::setParameter();
    std::vector<double> parameterVec; 
    featureTracker.m_camera[0]->writeParameters(parameterVec); 
    // k1, k2, p1, p2, fx, fy, cx, cy
    double fx = parameterVec[4]; 
    mbf = 0.1*fx; 
    cout << "estimator_dpt.cpp: rig length: 0.1 m, fx: "<< fx<<" mbf: "<<mbf<<endl; 

}

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

        if(!mb_calibrated)
        {
    	   handle_rgb(_img, img_undist);
    	   handle_dpt(dpt, dpt_align);
        }else{
            img_undist = _img.clone(); 
            dpt_align = dpt.clone();
        }

        // img_undist = _img.clone(); 
        // dpt_align = dpt.clone();
        featureFrame = featureTracker.trackImageDpt(t, mbf, img_undist, dpt_align);
    }
    //printf("featureTracker time: %f\n", featureTrackerTime.toc());

    if (SHOW_TRACK)
    {
        cv::Mat imgTrack = featureTracker.getTrackImage();
        pubTrackImage(imgTrack, t);
        // if(!featureTracker.mask_p.empty()){
        //    pubMaskImage(featureTracker.mask_p, t);
            // pubMaskPC(featureTracker.mask_pc, t);
        //}
    }
    
    if(MULTIPLE_THREAD)  
    {     
        // if(inputImageCnt % 2 == 0)
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
