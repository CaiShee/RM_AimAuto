#include <opencv2/core/types.hpp>
#if !defined(__WMIDENTIFY_HPP)
#define __WMIDENTIFY_HPP
#include "WMFunction.hpp"
#include "WMInference.hpp"
#include "globalParam.hpp"
#include "globalText.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <deque>
#include <glog/logging.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/video/video.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

class WMIdentify
{
private:
    int switch_INFO;    //<! 是否开启INFO级别的日志
    int switch_ERROR;   //<! 是否开启ERROR级别的日志
    int get_armor_mode; //<! 获取装甲板中心坐标的方法
    GlobalParam *gp;
    int t_start;
    cv::Mat img;                                            //<! 获取的原图，以及在调试过程中最终显示结果的背景图
    cv::Mat img_0;                                          //<! 获取的原图（仅对其用神经网络画框，不做其余操作）
    cv::Mat data_img;                                       //<! 转速与时间的数据图
    cv::Point2f R_emitate;
    cv::Mat binary;                                         //<! 经过预处理得到的图像
    std::vector<cv::Vec4i> hierarchy;                       //<! 获得的hierarchy关系，用于寻找装甲板
    std::vector<std::vector<cv::Point>> wing_contours;      //<! 扇页轮廓信息，用于寻找扇叶下半部分
    std::vector<cv::Vec4i> floodfill_hierarchy;             //<! 使用漫水处理方法时的hierarchy关系，暂时该方法被弃用
    std::vector<std::vector<cv::Point>> floodfill_contours; //<! 使用漫水处理方法时的轮廓信息，只能用来寻找装甲板，暂时该方法被弃用
    std::vector<std::vector<cv::Point>> R_contours;         //<! 寻找R使用的轮廓信息
    std::deque<cv::Point2d> wing_center_list;               //<! 装甲板中心点的队列
    std::deque<cv::Point2d> R_center_list;                  //<! R中心点的队列
    std::deque<int> wing_idx;                               //<! 扇叶的下半部分在wing_contours中的索引，返回并用作更新队列
    std::deque<int> winghat_idx;                            //<! 扇叶的上半部分在R_contours中的索引，返回并用作更新队列
    std::deque<int> R_idx;                                  //<! R的轮廓在R_contours中的索引，返回并用作更新队列
    int Wing_stat;                                          //<! 识别Wing的时候当前的状态，0为没有识别到Wing，1为识别到Wing，2为识别到多个Wing
    int Winghat_stat;                                       //<! 识别Winghat的时候当前的状态，0为没有识别到Wing，1为识别到Wing，2为识别到多个Wing
    int R_stat;                                             //<! 识别R的时候当前的状态，0为没有识别到R，1为识别到R，2为识别到多个R
    int list_stat;
    double direction;
    std::deque<double> time_list;           //<! 时间队列
    std::deque<double> angle_list;          //<! 角度队列
    std::deque<double> angle_velocity_list; //<! 角速度队列
    std::deque<double> R_yaw_list;
    WMDetector detector;

public:
    /**
     * @brief WMIdentify的构造函数，初始化一些参数
     *
     * @param gp 全局参数结构体，通过引用输入
     */
    WMIdentify(GlobalParam &);
    /**
     * @brief WMIdentify的析构函数，一般来说程序会自行销毁那些需要销毁的东西
     *
     */
    ~WMIdentify();
    /**
     * @brief 清空装甲板、R的中心点坐标以及索引的队列
     *
     */
    void clear();
    void startWMIdentify(cv::Mat &, Translator &);
    void startWMINet(cv::Mat &, Translator &);
    /**
     * @brief 输入图像的接口
     *
     * @param input_img 输入的图像
     */
    void receive_pic(cv::Mat &);
    /**
     * @brief 对图像进行预处理：蒙板，hsv二值化等操作
     *
     * @param gp 全局参数结构体，通过引用输入
     */
    void preprocess();
    /**
     * @brief 通过预处理后得到的binary图像进行膨胀腐蚀获得需要的图像，获得轮廓信息，这些轮廓信息将用于后续的寻找装甲板及R
     *
     * @param gp 全局参数结构体，通过引用输入
     */
    void getContours();
    /**
     * @brief 通过判断Wing的图形特征，获取若干Wing，并储存其索引在队列中。
     *
     * @param gp 全局参数结构体，通过引用输入
     * @return int 0为没有找到Wing，1为找到了一个Wing，2为找到了不止一个Wing
     */
    int getValidWingHalfHierarchy();
    /**
     * @brief 通过轮廓的拓扑关系筛选找到的若干Wing并留下真正的Wing的索引
     *
     * @return int 0代表选择失败，1代表选择成功
     */
    int selectWing();
    /**
     * @brief 通过漫水处理的方法找到装甲板，返回索引
     *
     * @param gp 全局参数结构体，通过引用输入
     * @return int 0代表失败，1代表成功
     */
    int getValidWingHatHierarchy();
    /**
     * @brief 通过判断Winghat的图形特征，获取若干Wing，并储存其索引在队列中。
     *
     * @return int 0为没有找到Winghat，1为找到了一个Winghat，2为找到了不止一个Winghat
     */
    int selectWingHat();
    /**
     * @brief 通过Winghat与Wing筛选找到的若干Winghat并留下真正的Winghat的索引
     *
     * @return int 0代表失败，1代表成功
     */
    int getArmorCenterFloodfill();
    /**
     * @brief 通过角点检测方法找到扇叶
     *
     * @return int 0代表失败，1代表成功
     */
    int getWingCenterapproxPolyDP();
    /**
     * @brief 通过判断R的图形特征，获取若干R，并储存其索引在队列中。
     *
     * @param gp 全局参数结构体，通过引用输入
     * @return int 若没有识别到R，返回0；识别到1个R，返回1；识别到多于一个R，返回2
     */
    int getVaildR();
    /**
     * @brief 在R_idx中若干索引中挑选真正的R的索引，通过的方法为判断R的坐标是否在中心区域
     *
     * @param gp 全局参数结构体，通过引用输入
     * @return int 0为失败，1为成功
     */
    int selectR();
    /**
     * @brief 通过索引，更新装甲板中心点与R中心点的队列
     *
     * @param gp 全局参数结构体，通过引用输入
     */
    void updateList(double);
    /**
     * @brief 获取时间队列
     *
     * @return std::deque<double> 时间队列
     */
    std::deque<double> getTimeList();
    /**
     * @brief 获取角速度队列
     *
     * @return std::deque<double> 角速度队列
     */
    std::deque<double> getAngleVelocityList();
    double getDirection();
    double getAngleList();
    double getYaw();
    int getListStat();
    cv::Mat getImg0();
    cv::Mat getData_img();
    cv::Point2d getR_center();
    double getRadius();
    void ClearSpeed();
};

#endif // __WMIDENTIFY_HPP