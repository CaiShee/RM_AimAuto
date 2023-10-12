#ifndef _PREDICT_HPP
#define _PREDICT_HPP
#include "globalParam.hpp"
#include <ceres/ceres.h>
#include <chrono>
#include <deque>
#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
class WMIPredict
{
private:
    double f1(double P0, double fly_t0, double theta_0, double v0);
    double f2(double P0, double fly_t0, double theta_0, double v0);
    double f1P(double P, double fly_t, double theta_0, double v0);
    double f1t(double P, double fly_t, double theta_0, double v0);
    double f2P(double P, double fly_t, double theta_0, double v0);
    double f2t(double P, double fly_t, double theta_0, double v0);

    double F1(double P0, double fly_t0, double theta_0, double v0);
    double F2(double P0, double fly_t0, double theta_0, double v0);
    double F1P(double P, double fly_t, double theta_0, double v0);
    double F1t(double P, double fly_t, double theta_0, double v0);
    double F2P(double P, double fly_t, double theta_0, double v0);
    double F2t(double P, double fly_t, double theta_0, double v0);
    double ThetaToolForBig(double dt, double t0);
    void SmoothData(std::deque<double> &y_data,std::deque<double> &x_data,std::deque<double> &y_data_s,std::deque<double> &x_data_s, int windowsize);
    void BSpline(std::deque<double> &x_data, std::deque<double> &y_data);
    cv::Point2d CalPointGuess(double theta);
    cv::Mat debugImg;
    cv::Mat data_img;
    cv::Mat smoothData_img;
    cv::Point2d R_center;
    std::deque<double> y_data_s;
    std::deque<double> x_data_s;
    double Radius;
    
    int direction;
    double Fire_time;
    double First_fit; //是否为初次拟合1，0
    //====大符速度参数======//
    double A0;
    double w_big;
    double b;
    double fai;
    double now_time;

public:
    WMIPredict();
    void UpdateData(double direction, double Radius,cv::Point2d R_center,cv::Mat  debugImg,cv::Mat data_img,Translator translator);
    int Fit(std::deque<double> time_list, std::deque<double> angle_velocity_list, GlobalParam &gp, Translator &tr);
    void NewtonDspSmall(double theta_0, Translator &translator, GlobalParam &gp, double R_yaw);
    void NewtonDspBig(double theta_0, Translator &translator, GlobalParam &gp, double R_yaw);
    void ResultLog(Translator &translator, GlobalParam &gp, double R_yaw);
    void GiveDebugImg(cv::Mat debugImg);
    cv::Mat GetDebugImg();
   
};

#endif // _PREDICT_HPP