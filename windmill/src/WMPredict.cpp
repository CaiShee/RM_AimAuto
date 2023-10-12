#include "WMPredict.hpp"
#include "SerialPort.hpp"
#include "WMFunction.hpp"
#include "WMIdentify.hpp"
#include "globalParam.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc/types_c.h"
#include <algorithm>
#include <ceres/loss_function.h>
#include <cmath>
#include <complex>
#include <deque>
//#include <fftw3.h>
#include <glog/logging.h>
#include <iostream>
#include <memory>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <ostream>
#include <unistd.h>
#include <vector>
//=====常量=====//
static double g = 9.8;         // 重力加速度
static double k = 0.05;        // 空气阻力系数
static double r = 0.7;         // 符的半径
static double s = 7;        // 车距离符的水平距离
//static double w = 1.047197551; // 小符角速度
static double w = 0; // 小符角速度
static double Exp = 2.71823; // 自然常数e
static double h0 = 1.1747;   // 符的中心点高度减去车的高度
static double pi = 3.14159;  // 圆周率
static double delta_t = 0.3;

struct CostFunctor
{

    CostFunctor(double time, double v_rad, double A, double w)
        : time_(time), v_rad_(v_rad), A_(A), w_(w) {}
    template <typename T>
    bool operator()(const T *const A, const T *const w, const T *const fai, T *residual) const
    {
        residual[0] = 5.0 * abs(v_rad_ - A[0] * sin(w[0] * time_ + fai[0]) - T(2.09) + A[0]);
        residual[1] = 10.0 * (abs(A[0] - 0.7) + abs(A[0] - 1.1) - 0.4);
        residual[2] = 10.0 * (abs(w[0] - 1.8) + abs(w[0] - 2.1) - 0.3);
        residual[3] = 5.0 * time_ / (time_ + 1) * (abs(A[0] - A_) + abs(w[0] - w_));
        return true;
    }
    const double time_;
    const double v_rad_;
    const double A_;
    const double w_;
};
WMIPredict::WMIPredict()
{
    this->direction = 1;
    this->smoothData_img = cv::Mat::zeros(400, 800, CV_8UC3);
    //====大符速度参数======//
    this->A0 = 1;
    this->w_big = 2;
    this->b = 2.09 - A0;
    this->fai = 0;
    this->now_time = 0;

    this->Fire_time = 0;
    this->First_fit = 1;
}
void WMIPredict::UpdateData(double direction, double Radius, cv::Point2d R_center, cv::Mat debugImg, cv::Mat data_img, Translator translator)
{
    this->direction = direction > 0 ? 1 : -1;
    this->R_center = R_center;
    this->Radius = Radius;
    this->now_time = (double)translator.message.predict_time / 1000;
    this->debugImg = debugImg;
    this->data_img = data_img;
}
int WMIPredict::Fit(std::deque<double> x_data, std::deque<double> y_data, GlobalParam &gp, Translator &tr)
{

    for (int i = 0; i < y_data.size(); i++)
    {
        y_data[i] = abs(y_data[i]);
    }
    // BSpline(x_data, y_data);
    // SmoothData(y_data, x_data, y_data_s, x_data_s, windowsize);

    ceres::Problem problem;
    double A_sk = this->A0;
    double w_sk = this->w_big;
    double fai_sk = this->fai;
    // int Size_que = std::min(time.size(),anglevelocity_rad.size());
    for (int i = 0; i < x_data.size(); i++)
    {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<CostFunctor, 4, 1, 1, 1>(
                new CostFunctor(x_data[i], y_data[i], this->A0, this->w_big)),
            nullptr,
            &A_sk, &w_sk, &fai_sk);
    }
    ceres::Solver::Options options;
    options.max_num_iterations = 30;
    //options.num_linear_solver_threads = 8;
    options.linear_solver_type = ceres::DENSE_QR;

    // options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;

    Solve(options, &problem, &summary);
    this->A0 = A_sk;
    this->b = 2.09 - A0;
    this->fai = fai_sk;
    this->w_big = w_sk;

#ifdef DEBUGHIT
    cv::Mat word_show = cv::Mat::zeros(600, 800, CV_8UC3);

    cv::putText(word_show, "time:" + std::to_string(tr.message.predict_time / 1000), cv::Point(30, 300), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 0));
    // cv::putText(word_show, "final_cost:" + std::to_string(sun), cv::Point(30, 330), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 0));

    cv::putText(word_show, "A0:" + std::to_string(this->A0), cv::Point(30, 420), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 0));

    cv::putText(word_show, "w0:" + std::to_string(this->w_big), cv::Point(30, 450), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 0));
    cv::putText(word_show, "fai0:" + std::to_string(this->fai), cv::Point(30, 480), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 0));
    cv::putText(word_show, "b:" + std::to_string(2.09 - this->A0), cv::Point(30, 510), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 0));
    cv::imshow("word_show", word_show);
#endif
    LOG_IF(INFO, gp.switch_INFO) << "estimated A:" << this->A0;
    LOG_IF(INFO, gp.switch_INFO) << "estimated w:" << this->w_big;
    LOG_IF(INFO, gp.switch_INFO) << "estimated sketchy:fai:" << this->fai << std::endl;

#ifdef DEBUGHIT
    // x_data.clear();
    // y_data.clear();
    // std::ifstream inputFile("/home/zxh/code.d/cmakeprojects/test/unbsp.txt");

    // double var1, var2;
    // std::string line;
    // while (std::getline(inputFile, line))
    // {
    //     std::istringstream iss(line);

    //     if (iss >> var1 >> var2)
    //     {
    //         static double count = 0;
    //         count++;
    //         x_data.push_back(var1);
    //         y_data.push_back(std::abs(var2));
    //     }
    // }
    // inputFile.close();
    std::stringstream ss;
    for (int i = 0; i < x_data.size(); i++)
    {
        ss << x_data[i] << " " << y_data[i] << std::endl;
    }
    static cv::Point2f first_point(x_data[0], abs(y_data[0]));

    for (int i = 0; i < x_data.size(); i++)
    {

        // cv::Point2f now_point(x_data_s[i], abs(y_data_s[i]));
        cv::Point2f now_point_fit(x_data[i], abs(y_data[i]));

        // now_point.x -= first_point.x;
        // now_point.x *= 20;
        // now_point.y = now_point.y * 100;

        now_point_fit.x -= first_point.x;
        now_point_fit.x *= 20;
        now_point_fit.y = (A0 * sin(w_big * x_data[i] + fai) + b) * 100;

        // cv::circle(this->smoothData_img, now_point, 1, cv::Scalar(0, 255, 255));
        // cv::circle(this->smoothData_img, now_point_fit, 1, cv::Scalar(0, 0, 255));
        cv::circle(this->data_img, now_point_fit, 1, cv::Scalar(0, 0, 255));
    }
    cv::imshow("initial", this->data_img);
    // cv::imshow("smooth", this->smoothData_img);
    std::ofstream file("/home/zxh/code.d/cmakeprojects/infantry_WM/points0.txt");
    file << ss.str();
#endif // DEBUGHIT
    return 1;
}
double WMIPredict::ThetaToolForBig(double dt, double t0) // 计算t0->t0+dt的大符角度
{
    return this->direction * (this->b * dt + this->A0 / this->w_big * (cos(this->w_big * t0 + this->fai) - cos(this->w_big * (t0 + dt) + this->fai)));
}
cv::Point2d WMIPredict::CalPointGuess(double theta)
{
    // 注意opencv坐标系
    cv::Point2d point_guess(R_center.x + Radius * cos(theta), R_center.y - Radius * sin((theta)));
    return point_guess;
}
double WMIPredict::f1(double P0, double fly_t0, double theta_0, double v0)
{
    return sqrt(pow(r * cos(theta_0 + w * fly_t0), 2) + s * s) - v0 * cos(P0) / k + v0 / k * cos(P0) * pow(Exp, -k * fly_t0);
}

double WMIPredict::f2(double P0, double fly_t0, double theta_0, double v0)
{
    return h0 + r * sin(theta_0 + w * fly_t0) - (k * v0 * sin(P0) + g - (k * v0 * sin(P0) + g) * pow(Exp, -k * fly_t0) - g * k * fly_t0) / (k * k);
}

double WMIPredict::f1P(double P, double fly_t, double theta_0, double v0) // f1关于p的导数
{
    return v0 * sin(P) / k * (1 - pow(Exp, -k * fly_t));
}
double WMIPredict::f1t(double P, double fly_t, double theta_0, double v0) // f1关于t的导数
{
    return (-r * r * w * cos(theta_0 + w * fly_t) * sin(theta_0 + w * fly_t)) / sqrt(pow(r * cos(theta_0 + w * fly_t), 2) + s * s) - v0 * cos(P) * pow(Exp, -k * fly_t);
}
double WMIPredict::f2P(double P, double fly_t, double theta_0, double v0) // f2关于p的导数
{
    return v0 * cos(P) / k * (pow(Exp, -k * fly_t) - 1);
}
double WMIPredict::f2t(double P, double fly_t, double theta_0, double v0) // f2关于t的导数
{
    return w * r * cos(theta_0 + w * fly_t) - (k * v0 * sin(P) + g) * pow(Exp, -k * fly_t) / k + g / k;
}

double WMIPredict::F1(double P0, double fly_t0, double theta_0, double v0)
{
    return sqrt(pow(r * cos(theta_0 + ThetaToolForBig(fly_t0, this->Fire_time)), 2) + s * s) - v0 * cos(P0) / k + v0 / k * cos(P0) * pow(Exp, -k * fly_t0);
}
double WMIPredict::F2(double P0, double fly_t0, double theta_0, double v0)
{
    return h0 + r * sin(theta_0 + ThetaToolForBig(fly_t0, this->Fire_time)) - (k * v0 * sin(P0) + g - (k * v0 * sin(P0) + g) * pow(Exp, -k * fly_t0) - g * k * fly_t0) / (k * k);
}
double WMIPredict::F1P(double P, double fly_t, double theta_0, double v0) // F1关于p的导数
{
    return v0 * sin(P) / k * (1 - pow(Exp, -k * fly_t));
}
double WMIPredict::F1t(double P, double fly_t, double theta_0, double v0) // f1关于t的导数
{
    return (-r * r * w * cos(theta_0 + ThetaToolForBig(fly_t, this->Fire_time)) * sin(theta_0 + ThetaToolForBig(fly_t, this->Fire_time))) / sqrt(pow(r * cos(theta_0 + ThetaToolForBig(fly_t, this->Fire_time)), 2) + s * s) - v0 * cos(P) * pow(Exp, -k * fly_t);
}
double WMIPredict::F2P(double P, double fly_t, double theta_0, double v0) // f2关于p的导数
{
    return v0 * cos(P) / k * (pow(Exp, -k * fly_t) - 1);
}
double WMIPredict::F2t(double P, double fly_t, double theta_0, double v0) // f2关于t的导数
{
    return w * r * cos(theta_0 + ThetaToolForBig(fly_t, this->Fire_time)) - (k * v0 * sin(P) + g) * pow(Exp, -k * fly_t) / k + g / k;
}

/**
 * @brief 牛顿迭代法求解子弹飞行时间
 *
 * @param P0 初始解 pitch
 * @param t0 初始解 飞行时间
 * @param theta_0 当前待打击点角度
 */
void WMIPredict::NewtonDspSmall(double theta_0, Translator &translator, GlobalParam &gp, double R_yaw)
{
    double P0 = 12 * pi / 180;
    double fly_t0 = 0.3;
    w = this->direction > 0 ? abs(w) : -abs(w);
    int n = 0;                                                                          // 迭代次数
    translator.message.predict_time = translator.message.predict_time + 1000 * delta_t; // 开火时间
    theta_0 = theta_0 + w * delta_t;                                                    // 开火时的待击打点角度
    theta_0 += theta_0 < 0 ? 2 * pi : 0;
    theta_0 -= theta_0 > 2 * pi ? 2 * pi : 0; // theta0 范围锁定（0，2pi）
    double v0 = translator.message.bullet_v;  // 弹速
    // std::cout << "v: " << v0 << std::endl;
    cv::Mat P_t = cv::Mat::zeros(2, 1, CV_64F);
    cv::Mat temp = (cv::Mat_<double>(2, 2) << this->f1P(P0, fly_t0, theta_0, v0), this->f1t(P0, fly_t0, theta_0, v0), this->f2P(P0, fly_t0, theta_0, v0), this->f2t(P0, fly_t0, theta_0, v0));
    cv::Mat temp_inv = cv::Mat::zeros(2, 2, CV_64F);
    cv::Mat b = (cv::Mat_<double>(2, 1) << this->f1(P0, fly_t0, theta_0, v0), this->f2(P0, fly_t0, theta_0, v0));
    double P1 = 0;
    double fly_t1 = 0;
    do
    {
        n++;
        P1 = P0;
        fly_t1 = fly_t0;
        // std::cout<<"P0:"<<P0；
        // std::cout<<" t0:"<<t0<<std::endl;
        //======这里对雅可比矩阵的更新要尽可能的少，不然解变化太快容易求出无意义解（t<0)======//
        temp.at<double>(0, 0) = this->f1P(P0, fly_t0, theta_0, v0);
        // temp.at<double>(0, 1) = f1t(P0, t0, theta_0,v0);
        // temp.at<double>(1, 0) = f2P(P0, t0, theta_0,v0);
        temp.at<double>(1, 1) = this->f2t(P0, fly_t0, theta_0, v0);
        // std::cout<<"temp: "<<temp<<std::endl;
        cv::invert(temp, temp_inv);
        // std::cout<<"temp_inv: "<<temp_inv<<std::endl;
        b.at<double>(0, 0) = this->f1(P0, fly_t0, theta_0, v0);
        b.at<double>(1, 0) = this->f2(P0, fly_t0, theta_0, v0);
        P_t = P_t - temp_inv * b;
        P0 = P_t.at<double>(0, 0);
        fly_t0 = P_t.at<double>(1, 0);
        if (n > 50)
            break;
    } while (abs(fly_t0 - fly_t1) > 1e-5 || abs(P0 - P1) > 1e-5); // 当前解与上次迭代解差距很小时

    double yaw = atan(r * cos(theta_0 + w * fly_t0) / s);
#ifdef DEBUGHIT

    double theta_guess = theta_0 + w * fly_t0;
    cv::circle(this->debugImg, CalPointGuess(theta_0), 5, cv::Scalar(0, 255, 255), -1);
    cv::circle(this->debugImg, CalPointGuess(theta_guess), 5, cv::Scalar(255, 0, 255), -1);

#endif
    translator.message.x_a = translator.message.yaw;
    translator.message.pitch = 180 * P0 / pi;
    translator.message.yaw = 180 * (yaw + R_yaw) / pi;
    // 相机中心的偏置修正
   // translator.message.yaw -= 1.1;
    // std::cout << " 开火时待打击点角度： " << 180 / pi * theta_0;
    LOG_IF(INFO, gp.switch_INFO) << "小符角速度" << w;
    LOG_IF(INFO, gp.switch_INFO) << " 开火时待打击点角度： " << 180 / pi * theta_0;
    LOG_IF(INFO, gp.switch_INFO) << " 对应的子弹飞行时间 " << fly_t0;
    LOG_IF(INFO, gp.switch_INFO) << " " << delta_t << "s后要调整的相对R的yaw: " << 180 / pi * yaw;
    LOG_IF(INFO, gp.switch_INFO) << " " << delta_t << "s后要调整的pitch: " << translator.message.pitch;
}
void WMIPredict::NewtonDspBig(double theta_0, Translator &translator, GlobalParam &gp, double R_yaw)
{
    double P0 = 12 * pi / 180;
    double fly_t0 = 0.3;
    int n = 0;                                                    // 迭代次数
    theta_0 = theta_0 + ThetaToolForBig(delta_t, this->now_time); // 开火时的待击打点角度
    this->Fire_time = this->now_time + delta_t;
    theta_0 += theta_0 < 0 ? 2 * pi : 0;
    theta_0 -= theta_0 > 2 * pi ? 2 * pi : 0;
    double v0 = translator.message.bullet_v; // 弹速
    cv::Mat P_t = cv::Mat::zeros(2, 1, CV_64F);
    cv::Mat temp = (cv::Mat_<double>(2, 2) << this->F1P(P0, fly_t0, theta_0, v0), this->F1t(P0, fly_t0, theta_0, v0), this->F2P(P0, fly_t0, theta_0, v0), this->F2t(P0, fly_t0, theta_0, v0));
    cv::Mat temp_inv = cv::Mat::zeros(2, 2, CV_64F);
    cv::Mat b = (cv::Mat_<double>(2, 1) << this->F1(P0, fly_t0, theta_0, v0), this->F2(P0, fly_t0, theta_0, v0));
    double P1 = 0;
    double fly_t1 = 0;
    do
    {
        n++;
        P1 = P0;
        fly_t1 = fly_t0;
        //======这里对雅可比矩阵的更新要尽可能的少，不然解变化太快容易求出无意义解（t<0)======//
        temp.at<double>(0, 0) = this->F1P(P0, fly_t0, theta_0, v0);
        // temp.at<double>(0, 1) = f1t(P0, t0, theta_0,v0);
        // temp.at<double>(1, 0) = f2P(P0, t0, theta_0,v0);
        temp.at<double>(1, 1) = this->F2t(P0, fly_t0, theta_0, v0);
        cv::invert(temp, temp_inv);
        b.at<double>(0, 0) = this->F1(P0, fly_t0, theta_0, v0);
        b.at<double>(1, 0) = this->F2(P0, fly_t0, theta_0, v0);
        P_t = P_t - temp_inv * b;
        P0 = P_t.at<double>(0, 0);
        fly_t0 = P_t.at<double>(1, 0);
        if (n > 50)
            break;
    } while (abs(fly_t0 - fly_t1) > 1e-5 || abs(P0 - P1) > 1e-5); // 当前解与上次迭代解差距很小时
    double yaw = atan(r * cos(theta_0 + ThetaToolForBig(fly_t0, this->Fire_time)) / s);
#ifdef DEBUGHIT
    double theta_guess = ThetaToolForBig(fly_t0, Fire_time) + theta_0;
    // double theta_fit_debug = ThetaToolForBig(2 * pi / w_big - delta_t, Fire_time) + theta_0;
    // cv::circle(this->debugImg, CalPointGuess(theta_fit_debug), 5, cv::Scalar(125, 125, 255), -1);
    cv::circle(this->debugImg, CalPointGuess(theta_0), 5, cv::Scalar(0, 255, 255), -1);
    cv::circle(this->debugImg, CalPointGuess(theta_guess), 5, cv::Scalar(255, 0, 255), -1);
    // cv::putText(this->debugImg, "points' distance:" + std::to_string(sqrt(CalDistSquare(CalPointGuess(theta_fit_debug), CalPointGuess(theta_0 - ThetaToolForBig(delta_t, this->now_time))))), cv::Point(30, 500), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 0));

#endif

    translator.message.x_a = translator.message.yaw;
    translator.message.pitch = P0 * 180 / pi;
    translator.message.yaw = (yaw + R_yaw) * 180 / pi;
   // translator.message.yaw -= 1.1;

    // std::cout<<"  speed_now: "<<A0*sin(w_big*(double)translator.message.predict_time/1000+fai)+this->b;
    translator.message.predict_time = translator.message.predict_time + 1000 * delta_t; // 发给电控的开火时间

    // std::cout<< " 对应的子弹飞行时间 " << fly_t0;
    // std::cout<< " " << delta_t << "s后要调整的相对R的yaw: " << 180 / pi * yaw;
    LOG_IF(INFO, gp.switch_INFO) << " 对应的子弹飞行时间 " << fly_t0;
    LOG_IF(INFO, gp.switch_INFO) << " " << delta_t << "s后要调整的相对R的yaw: " << 180 / pi * yaw;
    LOG_IF(INFO, gp.switch_INFO) << " " << delta_t << "s后要调整的pitch: " << translator.message.pitch;
    LOG_IF(INFO, gp.switch_INFO) << " 开火时待打击点角度： " << 180 / pi * theta_0;
    LOG_IF(INFO, gp.switch_INFO) << " 对应的子弹飞行时间 " << fly_t0;
    LOG_IF(INFO, gp.switch_INFO) << " " << delta_t << "s后要调整的相对R的yaw: " << 180 / pi * yaw;
    LOG_IF(INFO, gp.switch_INFO) << " " << delta_t << "s后要调整的pitch: " << translator.message.pitch;
}
void WMIPredict::ResultLog(Translator &translator, GlobalParam &gp, double R_yaw)
{
#ifdef DEBUGHIT
    cv::putText(this->debugImg, "bullet_v:" + std::to_string(translator.message.bullet_v), cv::Point(30, 60), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 100, 0));
    cv::putText(this->debugImg, "delta_t:" + std::to_string(delta_t), cv::Point(30, 90), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 100, 0));
    cv::putText(this->debugImg, "pitch:" + std::to_string(translator.message.pitch), cv::Point(30, 120), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 100, 0));
    cv::putText(this->debugImg, "delta_yaw:" + std::to_string(translator.message.yaw), cv::Point(30, 150), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 100, 0));
    cv::putText(this->debugImg, "R_yaw:" + std::to_string(R_yaw), cv::Point(30, 180), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 100, 0));
#endif
    LOG_IF(INFO, gp.switch_INFO) << "当前状态: " << +translator.message.status;
    LOG_IF(INFO, gp.switch_INFO) << "拍照时(" << translator.message.predict_time << ")云台相对于中心点R的yaw: " << 180 / pi * R_yaw;
    LOG_IF(INFO, gp.switch_INFO) << "预测时间戳: " << translator.message.predict_time;
}

void WMIPredict::SmoothData(std::deque<double> &y_data, std::deque<double> &x_data, std::deque<double> &y_data_s, std::deque<double> &x_data_s, int windowsize)
{
    // std::vector<double> smoothdata;
    double sum = 0.0;
    x_data_s = x_data;
    for (int i = 0; i < windowsize; i++)
    {
        sum += y_data[i];
        y_data_s.push_back(y_data[i]);
    }
    for (int i = windowsize; i < y_data.size(); i++)
    {
        sum = sum - y_data[i - windowsize] + y_data[i];
        y_data_s.push_back(sum / windowsize);
    }
}

// 暂时弃用
void WMIPredict::BSpline(std::deque<double> &x_data, std::deque<double> &y_data)
{
    // xy: 点向量，欲拟合的点，类型为double
    // x_data: 重新规划的x坐标

    std::vector<cv::Point2d> xy;
    for (int i = 0; i < x_data.size(); i++)
    {
        cv::Point2d temp(x_data[i], y_data[i]);
        xy.push_back(temp);
    }
    int n = xy.size();
    y_data.clear();
    double min_xdata = x_data[0];
    double max_xdata = x_data.back();
    double add_num = (max_xdata - min_xdata) / n;
    x_data.clear();
    for (int i = 0; i < n; i++)
    {
        x_data.push_back(min_xdata + i * add_num);
    }

    cv::Mat a = cv::Mat::zeros(n - 1, 1, CV_64FC1);

    cv::Mat b = cv::Mat::zeros(n - 1, 1, CV_64FC1);
    cv::Mat d = cv::Mat::zeros(n - 1, 1, CV_64FC1);
    cv::Mat dx = cv::Mat::zeros(n - 1, 1, CV_64FC1);
    cv::Mat dy = cv::Mat::zeros(n - 1, 1, CV_64FC1);
    for (int i = 0; i < xy.size() - 1; i++)
    {
        a.at<double>(i, 0) = xy[i].y;
        dx.at<double>(i, 0) = (xy[i + 1].x - xy[i].x);
        dy.at<double>(i, 0) = (xy[i + 1].y - xy[i].y);
    }
    cv::Mat A = cv::Mat::zeros(n, n, CV_64FC1);
    cv::Mat B = cv::Mat::zeros(n, 1, CV_64FC1);
    A.at<double>(0, 0) = 1;
    A.at<double>(n - 1, n - 1) = 1;
    for (int i = 1; i <= n - 2; i++)
    {
        A.at<double>(i, i - 1) = dx.at<double>(i - 1, 0);
        A.at<double>(i, i) = 2 * (dx.at<double>(i - 1, 0) + dx.at<double>(i, 0));
        A.at<double>(i, i + 1) = dx.at<double>(i, 0);
        B.at<double>(i, 0) = 3 * (dy.at<double>(i, 0) / dx.at<double>(i, 0) - dy.at<double>(i - 1, 0) / dx.at<double>(i - 1, 0));
    }
    cv::Mat c = A.inv() * B;
    for (int i = 0; i <= n - 2; i++)
    {
        d.at<double>(i, 0) = (c.at<double>(i + 1, 0) - c.at<double>(i, 0)) / (3 * dx.at<double>(i, 0));
        b.at<double>(i, 0) = dy.at<double>(i, 0) / dx.at<double>(i, 0) - dx.at<double>(i, 0) * (2 * c.at<double>(i, 0) + c.at<double>(i + 1, 0)) / 3;
    }
    int j = 0;
    for (int i = 0; i < x_data.size(); i++)
    {
        for (int ii = 0; ii <= n - 2; ii++)
        {
            if (x_data[i] >= xy[ii].x && x_data[i] < xy[ii + 1].x)
            {
                j = ii;
                break;
            }
            else if (x_data[i] >= xy[n - 1].x)
            {
                j = n - 2;
            }
        }
        double middleV = a.at<double>(j, 0) + b.at<double>(j, 0) * (x_data[i] - xy[j].x) + c.at<double>(j, 0) * (x_data[i] - xy[j].x) * (x_data[i] - xy[j].x) + d.at<double>(j, 0) * (x_data[i] - xy[j].x) * (x_data[i] - xy[j].x) * (x_data[i] - xy[j].x);
        y_data.push_back(middleV);
    }
}
cv::Mat WMIPredict::GetDebugImg()
{
    return this->debugImg;
}
void WMIPredict::GiveDebugImg(cv::Mat debugImg)
{
    this->debugImg = debugImg;
}