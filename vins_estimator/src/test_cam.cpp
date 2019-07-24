/*******************************************************
 *******************************************************/

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"

Estimator estimator;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_cam");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    if(argc != 2)
    {
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    readParameters(config_file);
    estimator.setParameter();

    cout<<"camera_parameter: "<<endl<<(estimator.featureTracker.m_camera[0]->parametersToString())<<endl;

    Eigen::Vector2d a(84.497238, 31.173565);
    Eigen::Vector3d b;
    estimator.featureTracker.m_camera[0]->liftProjective(a, b);

    cout<<"a = " <<a.transpose()<< endl<<" b = "<<b.transpose()<<endl;
    a << 7.637056, 157.970322;
    estimator.featureTracker.m_camera[0]->liftProjective(a, b);

    cout<<"a = " <<a.transpose()<< endl<<" b = "<<b.transpose()<<endl;
  
    // ros::spin();

    return 0;
}
