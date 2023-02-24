//
// Created by xytron on 23. 1. 6.
//
#include "lane_detection/lane_detection.h"

using namespace std;

int main(int argc, char ** argv){
    ros::init(argc, argv, "lane_detection");
    LaneDection ld;
    ros::spin();
    return 0;
}