//
// Created by xytron on 23. 1. 6.
//

#ifndef SRC_LANE_DETECTION_H
#define SRC_LANE_DETECTION_H

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <xycar_msgs/xycar_motor.h>
#include <std_msgs/Int32MultiArray.h>

#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

class LaneDection{
private:
    // ros initialization
    ros::NodeHandle nh;
    ros::NodeHandle pnh;

    ros::Subscriber subCam;
    ros::Subscriber subUltrasonic;

    ros::Publisher pubMotor;

    // Ultrasonic Process Variable & Function
    std_msgs::Int32MultiArray ultrasonic;
    bool b_ultrasonic;
    bool avoid;
    // Camera Process Variable & Function
    cv::Mat map1, map2;
    cv::Mat undistort_frame;

    std::vector<cv::Point2f> srcRect;
    std::vector<cv::Point2f> dstRect;

    std::vector<cv::Point2f> srcRect2;
    std::vector<cv::Point2f> dstRect2;

    void Cam_CB(const sensor_msgs::Image::ConstPtr& msg);
    void Ult_CB(const std_msgs::Int32MultiArray::ConstPtr& msg);

    cv::Mat calibrate(cv::Mat frame);
    cv::Mat return_bil(cv::Mat frame);
    cv::Mat return_canny(cv::Mat frame);
    cv::Mat return_roi(cv::Mat frame);
    cv::Mat return_grdient(cv::Mat frame);
    void show(std::string name, cv::Mat frame, int waitkey);
    void lane_classifivation(std::vector<cv::Vec4i> lines, cv::Mat frame);

    //state
    int cnt = 1 ;
    std::vector<int> SorC;
    void vec_updata(std::vector<int> &vec, int data, int maxsize);
    std::string SorC_check(std::vector<int> vec);
    std::vector<int> ult_vec;
    std::string ult_check(std::vector<int> vec);
public:
    LaneDection();
};

#endif //SRC_LANE_DETECTION_H
