//
// Created by xytron on 23. 1. 6.
//
#include "lane_detection/lane_detection.h"

using namespace cv;
using namespace std;

LaneDection::LaneDection()
:nh(""), pnh("~")
{
    subCam = nh.subscribe("/usb_cam/image_raw", 1, &LaneDection::Cam_CB, this);
    subUltrasonic = nh.subscribe("/xycar_ultrasonic", 1, &LaneDection::Ult_CB, this);
    pubMotor = nh.advertise<xycar_msgs::xycar_motor>("/xycar_motor", 1);

    /* ******************** */
    Mat cameraMatrix = (Mat1d(3,3)<< 836.256953716293,	0,	698.270739194534, 0,	837.291321267976,	385.047061537195, 0., 0., 1);
    Mat distCoeffs= (Mat1d(1,5)<<-0.346484839916854,0.147860778012205, 0, 0, -0.0335044200603601);
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), cameraMatrix,Size(1280,720),CV_32FC1, map1, map2);
    srcRect={Point2f(183,100), Point2f(93,475), Point2f(413, 479), Point2f(320,100)};
    dstRect={Point2f(160,0), Point2f(160, 480), Point2f(320, 480), Point2f(320,0)};
    srcRect2={Point2f(0,0), Point2f(0,480), Point2f(480, 480), Point2f(480,0)};
    dstRect2={Point2f(-120,0), Point2f(-120, 480), Point2f(700, 480), Point2f(700,0)};
    avoid = true;
    b_ultrasonic = false;
}
void LaneDection::Ult_CB(const std_msgs::Int32MultiArray::ConstPtr &msg) {
    ultrasonic = *msg;
    if (b_ultrasonic == false)
        b_ultrasonic =true;

    if ((ultrasonic.data[2] < 60 )){
        ROS_WARN("Dangerous[Collapse]");
        if (ultrasonic.data[2] < 57){
            ROS_WARN("Front sensor detect : %d", ultrasonic.data[2]);
        }
        if (ultrasonic.data[1] < 30){
            ROS_WARN("Front  Left sensor detect: %d", ultrasonic.data[1]);
        }
        if (ultrasonic.data[3] < 30){
            ROS_WARN("Front Right sensor detect: %d",ultrasonic.data[3]);
        }

        xycar_msgs::xycar_motor Motor;
        Motor.angle = 0;
        Motor.speed = 0;
        pubMotor.publish(Motor);
        ros::Duration(1).sleep();


    }

}
void LaneDection::Cam_CB(const sensor_msgs::Image::ConstPtr &msg) {
    Mat frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
    undistort_frame = calibrate(frame);
    resize(undistort_frame, undistort_frame, Size(640, 480), INTER_LINEAR);

    //show("yellow", yellow,1);


    GaussianBlur(undistort_frame,undistort_frame,Size(5,5),3);
    Mat bil = return_bil(undistort_frame);
    bil = return_bil(bil);
    Mat roi = return_roi(bil);

    resize(roi,roi,Size(480,480));


    Mat canny = return_canny(roi);
    Mat gradient = return_grdient(roi);



    Mat sum ;
    bitwise_and(canny, gradient, sum);
    Mat matx = getPerspectiveTransform(srcRect, dstRect);
    warpPerspective(sum, sum, matx, Size(roi.cols,roi.rows));
    matx = getPerspectiveTransform(srcRect2, dstRect2);
    warpPerspective(sum, sum, matx, Size(roi.cols,roi.rows));

    matx = getPerspectiveTransform(dstRect2, srcRect2);
    warpPerspective(sum, sum, matx, Size(roi.cols,roi.rows));
    matx = getPerspectiveTransform(dstRect, srcRect);
    warpPerspective(sum, sum, matx, Size(roi.cols,roi.rows));

    resize(sum,sum,Size(480,154));
    resize(roi,roi,Size(480,154));
//    Point triangle_point[3];
//    triangle_point[0] =Point(100,50);
//    triangle_point[1] =Point(100,153);
//    triangle_point[2] =Point(340,100);
//
//    const Point* ppt[1] = {triangle_point};
//    int npt[] ={3};
//    fillPoly(sum, ppt, npt, 1,  Scalar(0),LINE_AA);
    vector<Vec4i> linesP;
    //// HoughLinesP : threshold(만나는 점의 기준), minLineLength(선의 최소길이), maxLineGap(최대허용간격)
    HoughLinesP(sum, linesP, 1, (CV_PI/180), 30, 10, 20); // 그나마 잘 되었던 값 : (20,10,20) (15,10,20) (10,10,20)
    lane_classifivation(linesP, roi);

    imshow("sum", sum);
    //imshow("frame", sum);
    waitKey(1);
}
Mat LaneDection::calibrate(cv::Mat frame) {
    Mat tmp;
    remap(frame, tmp, map1, map2, INTER_LINEAR);
    return tmp;
}
Mat LaneDection::return_bil(cv::Mat frame) {
    Mat tmp;
    bilateralFilter(frame, tmp, 3, 100, 100);
    return tmp;
}
Mat LaneDection::return_canny(cv::Mat frame) {
    Mat tmp;
    Canny(frame, tmp, 80,125);
    return tmp;
}
Mat LaneDection::return_roi(cv::Mat frame) {
    Rect TmpBox(0,0,frame.cols, frame.rows);
    Rect r(0, 325 , frame.cols, 154 );
    return frame(r&TmpBox);
}
Mat LaneDection::return_grdient(cv::Mat frame) {
    Mat gray, scharr,  scharr_flip;
    cvtColor(frame, gray, COLOR_BGR2GRAY);

    Mat flied_gray;
    flip(gray, flied_gray,1);
    Sobel(gray, scharr, FILTER_SCHARR, 1, 0);
    Sobel(flied_gray, scharr_flip, FILTER_SCHARR, 1, 0);
    flip(scharr_flip, scharr_flip,1);
    Mat good_scharr_x;
    bitwise_or(scharr, scharr_flip,good_scharr_x);
    Mat flied_gray_y, scharr_y, scharr_y_flip;
    flip(gray, flied_gray_y,0);
    Sobel(gray, scharr_y, FILTER_SCHARR, 0, 1);
    Sobel(flied_gray_y, scharr_y_flip, FILTER_SCHARR, 0, 1);
    flip(scharr_y_flip, scharr_y_flip,0);
    Mat good_scharr_y;
    bitwise_or(scharr_y, scharr_y_flip,good_scharr_y);

    Mat good_scharr ;
    bitwise_or(good_scharr_x, good_scharr_y,good_scharr);

    Mat bin_scharr;
    threshold(good_scharr,bin_scharr, 60, 255, THRESH_BINARY);
    return bin_scharr;

}
void LaneDection::show(std::string name, cv::Mat frame, int waitkey){
    imshow(name, frame);
    if (waitKey(waitkey) =='c'){
        imwrite("/home/xytron/catkin_ws/src/lane_detection/frame.jpg", frame);
    }


}
void LaneDection ::lane_classifivation(std::vector<cv::Vec4i> lines, cv::Mat frame) {
    vector<double> slopes;
    vector<Vec4i> filtered_lines;
    for (auto line : lines){
        double slope =0.;
        // x1, y1, x2, y2 ( x_min, y_min, x_max, y_max)  = line
        if (line[0] == line[2]){
            slope = 1000.0;
        }
        else {
            slope = double(line[3]- line[1])/ double (line[2]- line[0]);
        }
        //slope check
        if (0.51< abs(slope) ||(0.2< abs(slope)&&abs(slope) <0.3)   ){
            //cout << slope << endl;
            slopes.push_back(slope);
            filtered_lines.push_back(line);
        }

    }
    if (slopes.size() ==0)
        return;
    vector<Vec4i> left_lines;
    vector<Vec4i> right_lines;

    for (int i =0; i<slopes.size(); i++){
        Vec4i Line = filtered_lines[i];
        double slope = slopes[i];

        if (slope < 0 && Line[2] < 480/2){
            left_lines.push_back(Line);
        }
        else if ( slope> 0 && Line[0] > 480/2){
            right_lines.push_back(Line);
        }
    }
    Mat draw_line;
    frame.copyTo(draw_line);
    for (auto l : left_lines){
        line( draw_line, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 2,8);
    }
    for (auto l : right_lines){
        line( draw_line, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,255,255), 2,8);
    }

    double m_left =0, b_left = 0;
    double x_sum = 0.0, y_sum =0.0, m_sum =0;

    if (left_lines.size() != 0){
        for (auto l : left_lines) {
            x_sum += l[0] + l[2];
            y_sum += l[1] + l[3];
            if (l[2] != l[0]) {
                m_sum += double(l[3] - l[1]) / double(l[2] - l[0]);
            }
            else {
                m_sum += 0;
            }
        }
        double x_avg = x_sum/ (left_lines.size() *2);
        double y_avg = y_sum / (left_lines.size() *2);
        m_left = m_sum / left_lines.size();
        b_left = y_avg - m_left * x_avg;

        if (m_left != 0.0){
            int x1 = int((0.0- b_left) / m_left);
            int x2 = int((154- b_left) / m_left);

            line(draw_line, Point(x1,0), Point(x2, 154),Scalar(255,0,0), 2);

        }
    }

    double m_right =0, b_right = 0;
    x_sum = 0.0, y_sum =0.0, m_sum =0;

    if (right_lines.size() != 0){
        for (auto l : right_lines) {
            x_sum += l[0] + l[2];
            y_sum += l[1] + l[3];
            if (l[2] != l[0]) {
                m_sum += double(l[3] - l[1]) / double(l[2] - l[0]);
            }
            else {
                m_sum += 0;
            }
        }
        double x_avg = x_sum/ (right_lines.size() *2);
        double y_avg = y_sum / (right_lines.size() *2);
        m_right = m_sum / right_lines.size();
        b_right = y_avg - m_right * x_avg;

        if (m_right != 0.0){
            int x1 = int((0.0- b_right) / m_right);
            int x2 = int((154- b_right) / m_right);

            line(draw_line, Point(x1,0), Point(x2, 154),Scalar(255,255,0), 2);
        }
    }
    double x_left = 0., x_right = 0., prev_x_left = 0, prev_x_right =480;
    int L_ROW = 80;
    if (m_left ==0){
        x_left = prev_x_left;
    }
    else {
        x_left = int((L_ROW - b_left)/ m_left);
    }
    if (m_right ==0){
        x_right = prev_x_right;
    }
    else {
        x_right = int((L_ROW - b_right)/ m_right);
    }
    if (m_left == 0 && m_right !=0){
        x_left = x_right -240;
    }
    if (m_left !=0 && m_right ==0){
        x_right = x_left +240;
    }
    prev_x_left = x_left;
    prev_x_right = x_right;
    int x_midpoint = (x_left+ x_right)/2;
    int view_center = 480/2 -20  ;

    line(draw_line, Point(0,L_ROW), Point(480, L_ROW), (0,255,255),2);
    rectangle(draw_line, Point(x_left-5, L_ROW-5), Point(x_left+5, L_ROW+5),(0,255,0),4);
    rectangle(draw_line, Point(x_right-5, L_ROW-5), Point(x_right+5, L_ROW+5),(0,255,0),4);
    rectangle(draw_line, Point(x_midpoint-5, L_ROW-5), Point(x_midpoint+5, L_ROW+5),(255,0,0),4);
    rectangle(draw_line, Point(view_center-5, L_ROW-5), Point(view_center+5, L_ROW+5),(0,0,255),4);
    imshow("draw_line", draw_line);
    waitKey(1);
    xycar_msgs::xycar_motor Motor;
    Motor.angle = int((x_midpoint-view_center) /4 );
    ROS_INFO("Diff : %d" , x_midpoint-view_center);
    if (abs(Motor.angle) <10 ){
        vec_updata(SorC, 0 , 30);
    }
    else {
        vec_updata(SorC, 1, 30 );
    }
    if (SorC.size()<6){
        return ;
    }
    else {
        if (b_ultrasonic == true){
            vec_updata(ult_vec, ultrasonic.data[2], 2);
            if (ult_vec.size()<2){
                return;
            }
            ROS_INFO("ultrasonic : %d, %d, %d", ultrasonic.data[1],ultrasonic.data[2],ultrasonic.data[3]);

            if (avoid == false ){
                //
                if ((ultrasonic.data[2] < 50 )){
                    ROS_WARN("Dangerous[Collapse]");
                    if (ultrasonic.data[2] < 50){
                        ROS_WARN("Front sensor detect : %d", ultrasonic.data[2]);
                    }
                    if (ultrasonic.data[1] < 30){
                        ROS_WARN("Front  Left sensor detect: %d", ultrasonic.data[1]);
                    }
                    if (ultrasonic.data[3] < 30){
                        ROS_WARN("Front Right sensor detect: %d",ultrasonic.data[3]);
                    }

                    xycar_msgs::xycar_motor Motor;

                    ros::Duration(1).sleep();
                    Motor.angle = +40;
                    Motor.speed = -20;
                    pubMotor.publish(Motor);
                    ros::Duration(2).sleep();
                    Motor.angle = -30;
                    Motor.speed = 13;
                    pubMotor.publish(Motor);
                    ros::Duration(1).sleep();
                    Motor.angle = 40;
                    Motor.speed = 12;
                    pubMotor.publish(Motor);
                    ros::Duration(2).sleep();
                    Motor.angle = -20;
                    Motor.speed = 5;
                    pubMotor.publish(Motor);
                    ros::Duration(1).sleep();
                    avoid =true;

                }
                else {
                    if (SorC_check( SorC) == "straight"){
                        Motor.speed = 11;
                        if (Motor.angle >0 ){
                            Motor.angle += 0;}
                        else {
                            Motor.angle -=5;
                        }
                     }
                    else if (SorC_check(SorC) == "curve") {
                        if (Motor.angle >0 ){
                            Motor.angle += 45;}
                        else {
                            Motor.angle -=15;
                        }
                        Motor.speed = 7;
                    }
                }
            }
            else {
                if (SorC_check(SorC) == "straight"){
                Motor.speed = 15;
                if (Motor.angle >0 ){
                    Motor.angle += 0;

                }
                else {
                    Motor.angle -=5;
                }
            }
            else if (SorC_check(SorC) == "curve") {
                if (Motor.angle >0 ){
                    Motor.speed = 7;
                    Motor.angle += 45;}
                else {
                    Motor.angle -=15;
                    Motor.speed = 9;
                }

            }
        }
        }
        else {
            if (SorC_check(SorC) == "straight"){
                Motor.speed = 18;
                if (Motor.angle >0 ){
                    Motor.angle += 0;}
                else {
                    Motor.angle -=5;
                }
            }
            else if (SorC_check(SorC) == "curve") {
                if (Motor.angle >0 ){
                    Motor.angle += 35;}
                else {
                    Motor.angle -=8;
                }
                Motor.speed = 7;
            }
        }
    }
    pubMotor.publish(Motor);
    cout << "steer : " << Motor.angle<<endl;
    cout << "speed : " << Motor.speed <<endl;
    cout <<"==========================="<<endl;
    //cout << b_ultrasonic <<endl;
    /*
    if (b_ultrasonic == true && (ultrasonic.data[2] < 55 || ultrasonic.data[1] < 45 || ultrasonic.data[3] < 45) && avoid ==false){
        ROS_WARN("Dangerous[Collapse]");
        xycar_msgs::xycar_motor Motor;
        Motor.angle = 0;
        Motor.speed = 0;
        pubMotor.publish(Motor);
        ros::Duration(0.5).sleep();
        Motor.angle = +40;
        Motor.speed = -20;
        pubMotor.publish(Motor);
        ros::Duration(2).sleep();
        Motor.angle = -30;
        Motor.speed = 13;
        pubMotor.publish(Motor);
        ros::Duration(1).sleep();
        Motor.angle = 40;
        Motor.speed = 10;
        pubMotor.publish(Motor);
        ros::Duration(2).sleep();
        Motor.angle = -20;
        Motor.speed = 5;
        pubMotor.publish(Motor);
        ros::Duration(1).sleep();
        avoid =true;
    }
    else {

        if (abs(Motor.angle) > 12) {
            if (Motor.angle >0 ){
            Motor.angle += 15;}
            else {
                Motor.angle -=10;
            }
            Motor.speed = 7;

        }

        else {
            if (avoid ==false){
                Motor.speed =11;
            }
            else {
                Motor.speed = 20;
            }
        }

    }

    pubMotor.publish(Motor);
    cout << "steer : " << Motor.angle<<endl;
    cout << "speed : " << Motor.speed <<endl;
    cout <<"==========================="<<endl;
     */
}
std::string LaneDection::SorC_check(std::vector<int> vec) {
    double sum=0.;
    for (auto i: vec){
        sum += i;
    }
    sum /= 30.;
    if (round(sum) == 0){
        ROS_INFO("Straight");
        return "straight";
    }
    else if (round(sum) == 1){
        ROS_INFO("Curve");
        return "curve";
    }
    else {
        ROS_ERROR("Junk value");
    }
}

void LaneDection::vec_updata(std::vector<int> &vec, int data, int maxsize) {
    if (vec.size() < maxsize){
        ROS_WARN("Vector Initializing : %d", vec.size());
        vec.push_back(data);
    }
    else {
        vec.erase(vec.begin());
        vec.push_back(data);
        ROS_INFO("Vector Updated : %d", vec.size());
    }

}
std::string LaneDection::ult_check(std::vector<int> vec) {
    double dif = vec[1] - vec[0] +  80;
    if (dif <0 || vec[1] <65){
        ROS_WARN("Collapse warning: %lf", dif);
        return "stop";
    }
    else {
        ROS_INFO("ULT : %lf", dif);
        return "go";
    }
}