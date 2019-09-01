/*
	Aug. 23, 2019, He Zhang, hzhang8@vcu.edu 

	test aligned structure core model 

*/

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "../utility/visualization.h"
#include "mask_people.h"

queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
std::mutex m_buf;

extern void handle_dpt(const cv::Mat& dpt, cv::Mat& dpt_out);
extern void handle_rgb(const cv::Mat& rgb, cv::Mat& rgb_out);

void handleImageDpt(double t, const cv::Mat &_img, const cv::Mat &dpt);

void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
    ROS_INFO("receive img0_call back!"); 
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
    ROS_INFO("receive img1_call back");
}

cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    cv::Mat ret_img;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        ret_img = ptr->image.clone();
    }
    else if(img_msg->encoding == "8UC3"){
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "bgr8"; // bgr
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8); // RGB
        ret_img = ptr->image.clone(); 
        // cv::cvtColor(ret_img, ret_img, cv::COLOR_BGR2GRAY);
    }
    else{
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
        ret_img = ptr->image.clone();
    }
    
    return ret_img;
}

cv::Mat getDepthImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "16UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono16";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO16);

    cv::Mat img = ptr->image.clone();
    return img;
}

// extract images with same timestamp from two topics
void sync_process()
{
    while(1)
    {
        if(STEREO)
        {
            cv::Mat image0, image1;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec();
                ROS_INFO("get time0 %lf time1 %lf", time0, time1);
                // 0.003s sync tolerance
                if(time0 < time1 - 0.003)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1 + 0.003)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }
                else
                {
                    time = img0_buf.front()->header.stamp.toSec();
                    header = img0_buf.front()->header;
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                    image1 = getDepthImageFromMsg(img1_buf.front());
                    img1_buf.pop();
                    printf("find img0 and img1\n");
                }
            }
            m_buf.unlock();
            if(!image0.empty())
                handleImageDpt(time, image0, image1);
        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_structure_core");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    STEREO = 1;
    registerPub(n);

    ros::Subscriber sub_img0 = n.subscribe("/cam0/color", 100, img0_callback);
    ros::Subscriber sub_img1 = n.subscribe("/cam0/depth", 100, img1_callback);

    std::thread sync_thread{sync_process};
    ros::spin();

    return 0;
}


void handleImageDpt(double t, const cv::Mat &_img, const cv::Mat &dpt)
{
   	cv::Mat dpt_align; 
    cv::Mat img_undist; 

    handle_rgb(_img, img_undist);
    handle_dpt(dpt, dpt_align);

    pubTrackImage(img_undist, t); 
    pubDepthImage(dpt_align, t);
    ros::spinOnce();

    sensor_msgs::PointCloud2 pc; 
    static MaskPeople* pm = new MaskPeople(); 

    // generate point cloud 
    cv::Mat mask_(img_undist.rows, img_undist.cols, CV_8UC1, cv::Scalar(0)); 
    pm->getMaskPts(img_undist, dpt_align, mask_, pc); 

    // publish 
    pubMaskPC(pc, t);
    ros::spinOnce(); 
    return ; 

}