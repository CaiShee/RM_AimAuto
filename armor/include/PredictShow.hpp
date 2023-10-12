#ifndef _PREDICT_SHOW_
#define _PREDICT_SHOW_

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

class PredictShow
{
typedef std::pair<cv::Point,double> pre;
private:
    /* data */
public:
    PredictShow(/* args */);
    void recvStatus(cv::Mat &src, double nowTime, cv::Point predictPoint, double flightTime);

public:
    std::vector<pre> pres;
    double thresh;
};

#endif
