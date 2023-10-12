#include "CameraParams.h"
#include "Eigen/Eigen"
#include "Eigen/src/Core/Matrix.h"
#include "opencv2/calib3d.hpp"
#include <cmath>
#include <cstdio>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ostream>
#include <string>
#include "../include/PredictShow.hpp"
#include <iostream>

PredictShow::PredictShow()
{
    this->thresh = 10;
}

void PredictShow::recvStatus(cv::Mat &src, double nowTime, cv::Point prePoint, double flighttime)
{
    double pre_Time = nowTime + flighttime;
    pre tmp = {prePoint, pre_Time};
    this->pres.push_back(tmp);
    std::vector<pre> tmps;

    for (auto j : pres)
    {
        if (abs(j.second - nowTime) < this->thresh)
        {
            cv::circle(src, j.first, 5, cv::Scalar(255, 255, 255), 5);
            std::cout<<"hihihihihi"<<std::endl;
        }
        else
        {
            tmps.push_back(j);
        }
    }

    this->pres=tmps;
}