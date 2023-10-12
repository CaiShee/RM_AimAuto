#ifndef AIMAUTO
#define AIMAUTO
#include "gaoning.hpp"
#include "globalParam.hpp"
#include "globalText.hpp"
#include "tracker.hpp"
#include <algorithm>
#include <camera.hpp>
#include <chrono>
#include <cmath>
#include <detector.hpp>
#include <fstream>
#include <map>
#include <memory>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/video.hpp>
#include <string>
#include <utility>
#include <vector>
#include "PredictShow.hpp"
class AimAuto
{
private:
    GlobalParam *gp;
    address addr;
    cv::Mat tVec, rVec;
    std::vector<float> _dist;
    cv::Mat _K;
    std::deque<Armor> tar_list;
    double dt;
    double r;
    double last_time, time, restart_time;
    Armors armors_msg;
    Tracker *tracker;
    double cnt;
    gaoning gn;
    bulletToGaoNing_t timer;
    std::unique_ptr<rm_auto_aim::Detector> det;
    void pnp_solve(rm_auto_aim::Armor &armor, Translator &ts, cv::Mat &src, Armor &tar, int number);
    cv::Point3f findTarget(Translator &ts, double pz, double pyaw, double &real_time);
    void storeMessage(cv::Point3f target, Translator &ts);
    void convertPoint(Translator &ts, cv::Point3f target);
    void fireControl(Translator &ts, cv::Mat &src);
    void showDist(std::deque<Armor> &tar_list, cv::Mat &src);
    bool updateTracker(Translator &ts, cv::Mat &src);

public:
    AimAuto(GlobalParam *gp);
    ~AimAuto();
    void AimAutoYHY(cv::Mat &src, Translator &ts);
    void NewTracker(Translator &ts, cv::Mat &src);
    PredictShow preshow;
    double flt;
    // void Tranform(cv::Mat &src,std::vector<rm_auto_aim::Armor> &armors,Translator &ts);
};
std::unique_ptr<rm_auto_aim::Detector> initDetector(int color);
#endif // AIMAUTO