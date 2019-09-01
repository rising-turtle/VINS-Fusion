/*
	July 23, 2019, He Zhang, hzhang8@vcu.edu 
	
	undistort stcture core's rgb data &
	align its depth data 

*/

#include <iostream>
#include <algorithm>
#include <fstream>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>  
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ros/ros.h>

using namespace std; 
using namespace cv; 

#define SQ(x) ((x)*(x))

void handle_rgb(const cv::Mat& rgb, cv::Mat& rgb_out)
{
  // undistortion 

  static float dist_data[5] = {0.320760, -0.864822, 0, 0, 0.589437}; 
  static cv::Mat distCoeffs(1, 5, CV_32F, dist_data);  
  static float cam_matrix_data[9] = {444.277, 0, 324.055, 0, 444.764, 254.516, 0, 0, 1};  
  static cv::Mat cameraMatrix(3, 3, CV_32F, cam_matrix_data); 

  cv::undistort(rgb, rgb_out, cameraMatrix, distCoeffs); 

  ROS_DEBUG("align_struct_core: rgb_out size %d x %d", rgb_out.cols, rgb_out.rows); 
}

void handle_dpt(const cv::Mat& dpt, cv::Mat& dpt_out)
{
  // construct 3d points 
  vector<Point3f> pts;
  pts.reserve(dpt.rows * dpt.cols);  
  float fx, fy, cx, cy; // not distortion for depth camera 

  // depth camera
  fx = 556.875; fy = 556.875; cx = 295.5; cy = 232.25; 
  float z,x,y ; 
  for(int r = 0; r<dpt.rows; r++){
    for(int c = 0; c<dpt.cols; c++){
      z = dpt.at<unsigned short>(r,c) * 0.001; 
      if(z >= 0.3 && z <= 7){ // range of structure core
        x = ((c - cx)/fx) * z; 
        y = ((r - cy)/fy) * z; 
        pts.push_back(Point3f(x, y, z)); 
      }
    }
  }

  // transform into color reference 
  static Eigen::Matrix4f Tc2d; // really //Td2c; 
  Tc2d << 0.999958, -0.00764741, 0.00501189, 0.0209042,
    0.00772149, 0.999859, -0.0149321, -8.10772e-05,
    -0.00489699, 0.0149701, 0.999876, -0.00251056,
    0, 0, 0, 1; 
  // static Eigen::Matrix4f Tc2d = Td2c.inverse(); // ??

  for(int i=0; i<pts.size(); i++){
    Point3f& pt = pts[i]; 
    Eigen::Vector4f pt_d(pt.x, pt.y, pt.z, 1.0); 
    Eigen::Vector4f pt_c = Tc2d * pt_d; 
    pt = Point3f(pt_c(0), pt_c(1), pt_c(2)); 
  }
  vector<Point2f> pts_2d(pts.size()); 

  // color camera default undistorted camera intrinsic parameters
  fx = 444.277; 
  fy = 444.764; 
  cx = 324.055; 
  cy = 254.516; 

  for(int i=0; i<pts.size(); i++){
    Point2f& pt_2d = pts_2d[i];
    Point3f& pt = pts[i];  
    pt_2d.x = (pt.x/pt.z)*fx + cx; 
    pt_2d.y = (pt.y/pt.z)*fy + cy; 
  }

  // ROS_INFO("point 3d %d point 2d : %d", pts.size(), pts_2d.size());
/*
  // project pts into image 
  float cam_matrix_data[9] = {444.277, 0, 324.055, 0, 444.764, 254.516, 0, 0, 1};  
  cv::Mat cameraMatrix(3, 3, CV_32F, cam_matrix_data); 
  float dist_data[5] = {0,0,0,0,0}; //{0.320760, -0.864822, 0, 0, 0.589437}; 
  cv::Mat distCoeffs(1, 5, CV_32F, dist_data);  
  float rvec_data[3] = {0,0,0}; 
  cv::Mat rvec(3,1, CV_32F, rvec_data); 
  float tvec_data[3] = {0,0,0}; 
  cv::Mat tvec(3,1, CV_32F, tvec_data); 

  projectPoints(pts, rvec, tvec, cameraMatrix, distCoeffs, pts_2d);
  ROS_DEBUG("after projectPoints");


  float rvec_data[3] = {0,0,0}; 
  cv::Mat rvec(3,1, CV_32F, rvec_data); 
  float tvec_data[3] = {0,0,0}; 
  cv::Mat tvec(3,1, CV_32F, tvec_data); 
  cam->projectPoints(pts, rvec, tvec, pts_2d); 
*/

  // 
  cv::Mat dpt_dis = cv::Mat(dpt.rows, dpt.cols, CV_32FC1, Scalar(0.0)); 
  cv::Mat dpt_cnt = cv::Mat(dpt.rows, dpt.cols, CV_32FC1, Scalar(0.0)); 
  // out_dpt = cv::Mat(dpt.cols, dpt.rows, CV_16UC1, Scalar) 
  int cnt = 0; 

  for(int i=0; i<pts_2d.size(); i++){
    float c = pts_2d[i].x; 
    float r = pts_2d[i].y; 

    if(c < 0 || r < 0 || c >= dpt.cols - 1 || r >= dpt.rows - 1)
      continue; 

    ++cnt; 

    float w; 
    int cl = floor(c); 
    int cr = ceil(c); 
    int ru = floor(r); 
    int rd = ceil(r);

    // left-up
    w = sqrt(SQ(1.0-( r -ru )) + SQ(1.0 - (c - cl))); 
    dpt_dis.at<float>(ru, cl) += w*pts[i].z; 
    dpt_cnt.at<float>(ru, cl) += w;
  
    // left-bottom
    w = sqrt(SQ(1.0 - (rd - r)) + SQ(1.0 - (c - cl))); 
    dpt_dis.at<float>(rd, cl) += w*pts[i].z; 
    dpt_cnt.at<float>(rd, cl) += w;

    // right-up
    w = sqrt(SQ(1.0-( r -ru )) + SQ(1.0 - (cr - c))); 
    dpt_dis.at<float>(ru, cr) += w*pts[i].z; 
    dpt_cnt.at<float>(ru, cr) += w;

    // right-bottom
    w = sqrt(SQ(1.0 - (rd - r)) + SQ(1.0 - (cr - c))); 
    dpt_dis.at<float>(rd, cr) += w*pts[i].z; 
    dpt_cnt.at<float>(rd, cr) += w;
        
  }

  // ROS_INFO("valid points number: %d", cnt);

  // do it 
  for(int r = 0; r<dpt.rows; r++){
    for(int c = 0; c<dpt.cols; c++){
      float w = dpt_cnt.at<float>(r, c) ; 
      if(w > 0)
        dpt_dis.at<float>(r,c) /= w; 
    }
  }

  // convert 
  dpt_dis.convertTo(dpt_out, CV_16UC1, 1000);
  ROS_DEBUG("align_struct_core: dpt_out size %d x %d", dpt_out.cols, dpt_out.rows); 
  return ;  
}