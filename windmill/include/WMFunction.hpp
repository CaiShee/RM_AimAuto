#if !defined(__WMFUNCTION_HPP)
#define __WMFUNCTION_HPP
#include "globalParam.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/video/video.hpp>
#include <iostream>
#include <glog/logging.h>
#include <Eigen/Dense>

/**
 * @brief 判断是否是可以用于hierarchy方法判断的扇叶下半部分
 *
 * @param contour 输入的轮廓
 * @param gp 全局变量结构体
 * @return true 是扇叶下半部分
 * @return false 不是扇叶下半部分
 */
bool IsVaildWingHalf(std::vector<cv::Point> contour, GlobalParam &gp);
bool IsVaildWingHat(std::vector<cv::Point> contour, GlobalParam &gp);
/**
 * @brief 判断是否是R
 *
 * @param contour 输入的轮廓
 * @param gp 全局变量结构体
 * @return true 是R
 * @return false 不是R
 */
bool IsVaildR(std::vector<cv::Point> contour, GlobalParam &gp);
/**
 * @brief 判断是否是通过漫水处理方法后得到的装甲板轮廓
 *
 * @param contour 输入的轮廓
 * @param gp 全局变量结构体
 * @return true 是装甲板
 * @return false 不是装甲板
 */
bool IsValidArmorFloodfill(std::vector<cv::Point> contour, GlobalParam &gp);
/**
 * @brief 通过输入的点与中心点坐标计算旋转角
 *
 * @param p1 点坐标
 * @param center 中心点坐标
 * @return double 返回弧度制角度
 */
double CalAngle(cv::Point p1, cv::Point center);
double CalDistSquare(cv::Point2f p1, cv::Point2f p2);
#endif // __WMFUNCTION_HPP