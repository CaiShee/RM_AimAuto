/**
 * @file WMIdentify.cpp
 * @author axi404 (3406402603@qq.com)
 * @brief 打符识别类实现，传统方法
 * @version 0.1
 * @date 2022-12-30
 *
 * @copyright Copyright (c) 2022
 */
#include "WMFunction.hpp"
#include "WMIdentify.hpp"
#include "globalParam.hpp"
#include "opencv2/core/hal/interface.h"
#include "opencv2/core/types.hpp"
#include <algorithm>
#include <deque>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <chrono>
#define Pi 3.1415926
WMIdentify::WMIdentify(GlobalParam &gp)
{
    this->gp = &gp;
    this->t_start = std::chrono::system_clock::now().time_since_epoch().count() / 1e9;
    // 从gp中读取一些数据
    this->switch_INFO = this->gp->switch_INFO;
    this->switch_ERROR = this->gp->switch_ERROR;
    this->get_armor_mode = this->gp->get_armor_mode;
    // 将R与Wing的状态设置为没有读取到内容
    this->R_stat = 0;
    this->Wing_stat = 0;
    this->Winghat_stat = 0;
    this->R_emitate.x = 0;
    this->R_emitate.y = 0;
    this->data_img = cv::Mat::zeros(400, 800, CV_8UC3);
    // 输出日志，初始化成功
    LOG_IF(INFO, this->switch_INFO) << "WMIdentify Successful";
}

WMIdentify::~WMIdentify()
{
    // WMIdentify之中的内容都会自动析构
    std::cout << "析构中，下次再见喵～" << std::endl;
    // 输出日志，析构成功
    LOG_IF(INFO, this->switch_INFO) << "~WMIdentify Successful";
}

void WMIdentify::clear()
{
    // 此内容一般不会使用
    // 清空队列中的内容
    this->wing_center_list.clear();
    this->wing_idx.clear();
    this->R_center_list.clear();
    this->R_idx.clear();
    this->time_list.clear();
    this->angle_list.clear();
    this->angle_velocity_list.clear();
    this->angle_velocity_list.emplace_back(0); // 先填充一个0，方便之后UpdateList中的数据对齐
    // 输出日志，清空成功
    LOG_IF(INFO, this->switch_INFO) << "clear Successful";
}

void WMIdentify::startWMIdentify(cv::Mat &input_img, Translator &ts)
{
    // 输入图片
    this->receive_pic(input_img);
    // 预处理
    this->preprocess();
    // 获取轮廓
    this->getContours();
    // 通过图像特征寻找扇页下半部分
    this->getValidWingHalfHierarchy();
    // 通过父子关系确定扇页下半部分
    this->selectWing();
    // 通过图像特征寻找扇页上半部分
    this->getValidWingHatHierarchy();
    // 通过与扇页下半部分距离确定扇页上半部分
    this->selectWingHat();
    // 通过图像特征寻找R
    this->getVaildR();
    // 通过图像位置确定R
    this->selectR();
    // 更新数据队列
    this->updateList((double)ts.message.predict_time / 1000);
}

void WMIdentify::startWMINet(cv::Mat &input_img, Translator &ts)
{
    // 输入图片
    this->receive_pic(input_img);
    std::vector<WMObject> objects;
     std::chrono::milliseconds start_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch());
    detector.detect(this->img, objects);
     
    this->list_stat = 0;
    for (auto object : objects)
    {

        // std::cout<<"cls: "<<object.cls<<std::endl;
        // std::cout<<object.color<<std::endl;
        if (object.cls == 0 && ((object.color == 0 && gp->color == RED) || (object.color == 0 && gp->color == BLUE)) && (object.cls >= 0 && object.cls <= 5))
        {
            LOG_IF(INFO, gp->switch_INFO) << "神经网络识别成功:" << object.cls;
#if defined(DEBUGMODE) or defined(DEBUGHIT)

            cv::line(this->img, object.apex[0], object.apex[1], cv::Scalar(193, 182, 255), 3);
            cv::line(this->img, object.apex[1], object.apex[2], cv::Scalar(193, 182, 255), 3);
            cv::line(this->img, object.apex[2], object.apex[3], cv::Scalar(193, 182, 255), 3);
            cv::line(this->img, object.apex[3], object.apex[0], cv::Scalar(193, 182, 255), 3);
            cv::circle(this->img, (object.apex[3] + object.apex[2] + object.apex[1]) / 3, 5, cv::Scalar(0, 0, 255), -1);
#endif // DEBUGMODE
            float K_ab = abs(object.apex[1].x - object.apex[3].x) > 0.00001 ? (object.apex[1].y - object.apex[3].y) / (object.apex[1].x - object.apex[3].x) : 100000;
            float AB_x_cte = (object.apex[1].x + object.apex[3].x) / 2;
            float AB_y_cte = (object.apex[1].y + object.apex[3].y) / 2;
            float length_AB = sqrt(CalDistSquare(object.apex[1], object.apex[3]));
            float E_x = AB_x_cte + 2.7468 * length_AB / sqrt(1 + 1 / pow(K_ab, 2));
            float E_y = -1 / K_ab * (E_x - AB_x_cte) + AB_y_cte;
            if (((object.apex[3].x - object.apex[2].x) * (E_x - object.apex[2].x) + (object.apex[3].y - object.apex[2].y) * (E_y - object.apex[2].y)) <= 0)
            {
                E_x = AB_x_cte - 2.7468 * length_AB / sqrt(1 + 1 / pow(K_ab, 2));
                E_y = -1 / K_ab * (E_x - AB_x_cte) + AB_y_cte;
            }
            if (this->wing_center_list.size() >= 2)
            {
                this->wing_center_list.pop_front();
            }
            // if (this->R_center_list.size() >= 2)
            // {
            //     this->R_center_list.pop_front();
            // }
            this->wing_center_list.push_back((object.apex[3] + object.apex[2] + object.apex[1]) / 3);
            this->R_emitate.x = E_x;
            this->R_emitate.y = E_y;
#ifdef DEBUGMODE
            // cv::circle(img, this->R_emitate, 5, cv::Scalar(255, 255, 255),-1);
#endif
            this->list_stat = 1;
        }
    }

    this->preprocess();

    // 获取轮廓
    this->getContours();

    // 通过图像特征寻找R
    this->getVaildR();
    // 通过图像位置确定R
    this->selectR();

    if (this->R_center_list.size() >= 2)
    {
        this->R_center_list.pop_front();
    }
    // 如果寻找R中心点确实找到东西了，列表不为空，则把找到的R中心点坐标压进队列中，否则输出日志，数据不足
    if (this->R_idx.size() > 0 && this->R_contours.size() > 0 && list_stat == 1)
    {
        this->R_center_list.emplace_back(cv::minAreaRect(this->R_contours[this->R_idx[0]]).center);
    }
    else
    {
        LOG_IF(ERROR, this->switch_ERROR) << "failed to emplace_back R_center_list, lack data";
        if (list_stat == 1)
        {
            // 此时armor已经输入了数据，但是R无法输入数据，所以要把armor的数据吐出来，保证数据同步
            this->wing_center_list.pop_back();
        }
        list_stat = 0;
    }
    this->updateList((double)ts.message.predict_time / 1000);
}

void WMIdentify::receive_pic(cv::Mat &input_img)
{
    this->img = input_img;

    LOG_IF(INFO, this->switch_INFO) << "receive_pic Successful";
}

void WMIdentify::preprocess()
{
    // NOTE: 将来优化代码可以减少克隆次数
    cv::Mat mask(img.rows, img.cols, CV_8UC1, cv::Scalar(0)); //!< 创建的单通道图像，用于蒙板
    cv::Mat dst1;
#ifdef DEBUGHIT
    this->img_0 = this->img.clone();

#endif // DEBUGHIT
    // 将蒙板中需要取到的区域（在globalParam中调整）内的像素点变为白色
    mask(cv::Rect(this->img.cols * this->gp->mask_TL_x, this->img.rows * this->gp->mask_TL_y, this->img.cols * this->gp->mask_width, this->img.rows * this->gp->mask_height)) = 255;
    // 实现蒙板，将两边与能量机关无关的部分去掉
    this->img.copyTo(dst1, mask); //!< 用于储存蒙板之后的图片

#ifdef DEBUGMODE
    // 展示蒙板之后的结果
    cv::imshow("after mask", dst1);
#endif // DEBUGMODE
    if (this->gp->switch_gaussian_blur == ON)
    {
        cv::GaussianBlur(dst1, dst1, cv::Size(5, 5), 0.0, 0.0);
    }
    // hsv二值化，此部分的参数对于后续的操作尤为重要

    cv::cvtColor(dst1, dst1, cv::COLOR_BGR2HSV);
    cv::inRange(dst1, cv::Scalar(this->gp->hmin, this->gp->smin, this->gp->vmin), cv::Scalar(this->gp->hmax, this->gp->smax, this->gp->vmax), this->binary);

#ifdef DEBUGMODE
    // 展示预处理之后的binary
    cv::imshow("binary after preprocess", this->binary);
#endif // DEBUGMODE

    // 日志输出预处理成功
    LOG_IF(INFO, this->switch_INFO == ON) << "preprocess successful";
}

void WMIdentify::getContours()
{
    // 清空轮廓信息
    this->R_contours.clear();
    this->wing_contours.clear();
    cv::Mat binary_clone = this->binary;
    // 进行图形学操作，最后新添加的膨胀后腐蚀可以让灯带连在一起
    cv::dilate(binary_clone, binary_clone, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(this->gp->dialte1, this->gp->dialte1)));
    // 检测最外围轮廓，储存找到的轮廓至R_contours，用于R的识别
    cv::findContours(binary_clone, this->R_contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

#ifdef DEBUGMODE
    cv::imshow("R", binary_clone);
#endif // DEBUGMODE
#ifndef USEWMNET
    // 进行获取装甲板需要的图形学操作
    cv::dilate(binary_clone, binary_clone, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(this->gp->dialte2, this->gp->dialte2)));
    cv::dilate(binary_clone, binary_clone, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(this->gp->dialte3, this->gp->dialte3)));
    // 提取binary中所有轮廓，且hierarchy是以“树状结构”来进行组织，储存找到的轮廓至wing_contours，用于装甲板的识别
    cv::findContours(binary_clone, this->wing_contours, this->hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
#endif
#ifdef DEBUGMODE
    cv::imshow("armor", binary_clone);
#endif // DEBUGMODE
    // 日志输出获得轮廓成功成功
    LOG_IF(INFO, this->switch_INFO == ON) << "getContours successful";
}

int WMIdentify::getValidWingHalfHierarchy()
{
    // 清空扇叶下半部分索引信息
    this->wing_idx.clear();
    for (int i = 0; i < this->wing_contours.size(); i++)
    {
        // 通过轮廓特征筛选出可能的扇叶下半部分，理论来说只会筛出来被激活和未被激活的扇叶，效果好直接筛掉被激活的扇叶
        if (IsVaildWingHalf(this->wing_contours[i], *gp))
        {
            // 如果判断为是，将索引压入索引队列
            this->wing_idx.emplace_back(i);
            // 日志输出找到装甲板
            LOG_IF(INFO, this->switch_INFO == ON) << "getValidWingHalfHierarchy successful once";
        }
    }
    // 通过当前找到的符合的Wing的数量，输出当前状态
    if (this->wing_idx.size() == 0)
    {
        LOG_IF(INFO, this->switch_INFO) << "getValidWingHalfHierarchy Get No Wing";
        this->Wing_stat = 0;
    }
    else if (this->wing_idx.size() == 1)
    {
        LOG_IF(INFO, this->switch_INFO) << "getValidWingHalfHierarchy Get One Wing";
        this->Wing_stat = 1;
    }
    else if (this->wing_idx.size() > 1)
    {
        LOG_IF(INFO, this->switch_INFO) << "getValidWingHalfHierarchy Get Many Wing:" << this->wing_idx.size();
        this->Wing_stat = 2;
    }
    return Wing_stat;
}

int WMIdentify::selectWing()
{
    // 没有索引，直接失败
    if (this->Wing_stat == 0)
    {
        LOG_IF(INFO, this->switch_INFO == ON) << "selectWing failed";
        return 0;
    }
    // 只有一个索引，认为找到了需要的扇叶下半部分
    else if (this->Wing_stat == 1)
    {
        // 如果UI开关打开
        if (this->gp->switch_UI == ON)
        {
            // 遍历idx里面每一个索引(其实只有一个，保持结构一致，不做更改)
            for (int i = 0; i < this->wing_idx.size(); i++)
            {
                // 如果轮廓显示开关打开，画出当前找到的扇叶的轮廓
                if (this->gp->switch_UI_contours == ON)
                    cv::drawContours(this->img, this->wing_contours, this->wing_idx[i], cv::Scalar(0, 255, 0), 3);
                // 如果面积显示开关打开，显示出当前找到的扇叶的面积、长宽比、面积比
                if (this->gp->switch_UI_areas == ON)
                {
                    double s_area = cv::contourArea(this->wing_contours[this->wing_idx[i]]);
                    cv::putText(this->img, std::to_string(s_area), cv::minAreaRect(this->wing_contours[this->wing_idx[i]]).center, 1, 1, cv::Scalar(255, 255, 0));
                    cv::RotatedRect armor_rect = minAreaRect(this->wing_contours[this->wing_idx[i]]);
                    cv::Size2f armor_size = armor_rect.size;
                    float length = armor_size.height > armor_size.width ? armor_size.height : armor_size.width; // 将矩形的长边设置为长
                    float width = armor_size.height < armor_size.width ? armor_size.height : armor_size.width;  // 将矩形的短边设置为宽
                    float lw_ratio = length / width;
                    float s_ratio = s_area / armor_size.area();
                    cv::putText(this->img, std::to_string(lw_ratio), cv::minAreaRect(this->wing_contours[this->wing_idx[i]]).center + cv::Point2f(0, 20), 1, 1, cv::Scalar(255, 255, 0));
                    cv::putText(this->img, std::to_string(s_ratio), cv::minAreaRect(this->wing_contours[this->wing_idx[i]]).center + cv::Point2f(0, 40), 1, 1, cv::Scalar(255, 255, 0));
                }
            }
        }
        LOG_IF(INFO, this->switch_INFO == ON) << "selectWing successful";
        return 1;
    }
    // 有很多的索引，开始筛选
    else if (this->Wing_stat == 2)
    {
        // 在筛选之前，先了解具体的内容，方便调参
        // 如果UI开关打开
        if (this->gp->switch_UI == ON)
        {
            // 遍历idx里面每一个索引
            for (int i = 0; i < this->wing_idx.size(); i++)
            {
                // 如果轮廓显示开关打开，画出当前找到的扇叶的轮廓
                if (this->gp->switch_UI_contours == ON)
                    cv::drawContours(this->img, this->wing_contours, this->wing_idx[i], cv::Scalar(0, 255, 0), 3);
                // 如果面积显示开关打开，显示出当前找到的扇叶的面积
                if (this->gp->switch_UI_areas == ON)
                {
                    double s_area = cv::contourArea(this->wing_contours[this->wing_idx[i]]);
                    cv::putText(this->img, std::to_string(s_area), cv::minAreaRect(this->wing_contours[this->wing_idx[i]]).center, 1, 1, cv::Scalar(255, 255, 0));
                    cv::RotatedRect armor_rect = minAreaRect(this->wing_contours[this->wing_idx[i]]);
                    cv::Size2f armor_size = armor_rect.size;
                    float length = armor_size.height > armor_size.width ? armor_size.height : armor_size.width; // 将矩形的长边设置为长
                    float width = armor_size.height < armor_size.width ? armor_size.height : armor_size.width;  // 将矩形的短边设置为宽
                    float lw_ratio = length / width;
                    float s_ratio = s_area / armor_size.area();
                    cv::putText(this->img, std::to_string(lw_ratio), cv::minAreaRect(this->wing_contours[this->wing_idx[i]]).center + cv::Point2f(0, 20), 1, 1, cv::Scalar(255, 255, 0));
                    cv::putText(this->img, std::to_string(s_ratio), cv::minAreaRect(this->wing_contours[this->wing_idx[i]]).center + cv::Point2f(0, 40), 1, 1, cv::Scalar(255, 255, 0));
                }
            }
        }
        // 进行筛选，原则是，我们认为当前轮廓没有父轮廓，没有实际有意义的子轮廓(有的子轮廓为小型的空洞噪点，称之为"气泡")，就是所需要的轮廓，而且这种轮廓在找到的队列之中至多有一个
        // 会持续的检测当前队列中的第0个轮廓是否符合要求，如果不符合要求就踢掉，然后检测下一个轮廓(此时下一个轮廓还是位于第0个)
        while (1)
        {
            int i = 1;     //<! 当前判断的轮廓为wing_idx[0]与wing_idx[0]+i这两个轮廓，之所以存在i，是为了保证当确定轮廓因为存在子轮廓被踢掉的时候，检索过程可以跨过那些气泡
            int flag = -1; //<! 当前标志位，flag=0意味着该轮廓是未被激活的扇叶下半部分，flag=-1意味着该轮廓不是所求
            while (1)
            {
                // 假如说轮廓有父轮廓，其一定不是需要的
                if (this->hierarchy[this->wing_idx[0]][3] != -1)
                {
                    flag = -1;
                    break;
                }
                // 当当前判定的子轮廓不为整体树状结构的最后一个时
                if (this->hierarchy.size() > this->wing_idx[0] + i)
                {
                    // 假如说下一个轮廓不是当前轮廓的子轮廓，则当前轮廓不包含任何的子轮廓，认为是未被激活的扇叶下半部分
                    if (this->hierarchy[this->wing_idx[0] + i][3] != this->wing_idx[0])
                    {
                        flag = 0;
                        break;
                    }
                    // 假如说轮廓有子轮廓，而且不是“气泡”，认为其可能是已经被激活的扇叶下半部分，排除
                    if (this->hierarchy[this->wing_idx[0] + i][3] == this->wing_idx[0] && cv::contourArea(this->wing_contours[this->wing_idx[0] + i]) > 50)
                    {
                        flag = -1;
                        break;
                    }
                }
                // 从当前轮廓向后数直到最后一个轮廓都不是子轮廓，已经到了树状结构的尽头，其一定没有子轮廓
                else
                {
                    flag = 0;
                    break;
                }
                // 假如以上都不符合，当前wing_idx[0] + j轮廓认为是一个“气泡”，则是否踢掉wing_idx[0]，取决于下一个轮廓，也就是wing_idx[0] + j + 1轮廓是否为非气泡子轮廓。
                i += 1;
            }
            // 第0个不为未被激活的扇叶下半部分，移除，查看下一个第0个
            if (flag != 0)
            {
                this->wing_idx.pop_front();
            }
            // 是未被激活的扇叶下半部分，第0个为未被激活的扇叶下半部分
            if (flag == 0)
            {
                break;
            }
            // 全部的轮廓都不符合要求，退出
            if (this->wing_idx.size() == 0)
            {
                break;
            }
        }
    }
    // 如果轮廓不为0个，其第0个轮廓是所需要的，成功
    if (this->wing_idx.size() != 0)
    {
        this->Wing_stat = 1;
        LOG_IF(INFO, this->switch_INFO == ON) << "selectWing successful";
    }
    // 如果轮廓为0个，全部轮廓都被排除，失败
    else if (this->wing_idx.size() == 0)
    {
        this->Wing_stat = 0;
        LOG_IF(INFO, this->switch_INFO == ON) << "selectWing failed No Wings";
    }
    return 1;
}

int WMIdentify::getValidWingHatHierarchy()
{
    // 清空扇叶上半部分索引队列
    this->winghat_idx.clear();
    for (int i = 0; i < this->R_contours.size(); i++)
    {
        // 通过轮廓特征筛选出可能的Winghat，值得一提的是使用R_contours，因为考虑到不需要考虑内轮廓
        if (IsVaildWingHat(this->R_contours[i], *gp))
        {
            this->winghat_idx.emplace_back(i);
            // 日志输出找到装甲板
            LOG_IF(INFO, this->switch_INFO == ON) << "getValidWingHalfHierarchy successful once";
        }
    }
    // 通过当前找到的符合的Wing的数量，输出当前状态
    if (this->winghat_idx.size() == 0)
    {
        LOG_IF(INFO, this->switch_INFO) << "getValidWingHatHierarchy Get No Winghat";
        this->Winghat_stat = 0;
    }
    else if (this->winghat_idx.size() == 1)
    {
        LOG_IF(INFO, this->switch_INFO) << "getValidWingHatHierarchy Get One Winghat";
        this->Winghat_stat = 1;
    }
    else if (this->winghat_idx.size() > 1)
    {
        LOG_IF(INFO, this->switch_INFO) << "getValidWingHatHierarchy Get Many Winghat:" << this->winghat_idx.size();
        this->Winghat_stat = 2;
    }
    return this->Winghat_stat;
}

int WMIdentify::selectWingHat()
{
    // 如果UI开关打开
    if (this->gp->switch_UI == ON)
    {
        // 遍历idx里面每一个索引
        for (int i = 0; i < this->winghat_idx.size(); i++)
        {
            // 如果轮廓显示开关打开，画出当前找到的扇叶上半部分的轮廓
            if (this->gp->switch_UI_contours == ON)
                cv::drawContours(this->img, this->R_contours, this->winghat_idx[i], cv::Scalar(255, 0, 0), 3);
            // 如果面积显示开关打开，显示出当前找到的扇叶的面积、长宽比、面积比
            if (this->gp->switch_UI_areas == ON)
            {
                double s_area = cv::contourArea(this->R_contours[this->winghat_idx[i]]);
                cv::putText(this->img, std::to_string(s_area), cv::minAreaRect(this->R_contours[this->winghat_idx[i]]).center, 1, 1, cv::Scalar(0, 255, 255));
                cv::RotatedRect armor_rect = minAreaRect(this->R_contours[this->winghat_idx[i]]);
                cv::Size2f armor_size = armor_rect.size;
                float length = armor_size.height > armor_size.width ? armor_size.height : armor_size.width; // 将矩形的长边设置为长
                float width = armor_size.height < armor_size.width ? armor_size.height : armor_size.width;  // 将矩形的短边设置为宽
                float lw_ratio = length / width;
                float s_ratio = s_area / armor_size.area();
                cv::putText(this->img, std::to_string(lw_ratio), cv::minAreaRect(this->R_contours[this->winghat_idx[i]]).center + cv::Point2f(0, 20), 1, 1, cv::Scalar(0, 255, 255));
                cv::putText(this->img, std::to_string(s_ratio), cv::minAreaRect(this->R_contours[this->winghat_idx[i]]).center + cv::Point2f(0, 40), 1, 1, cv::Scalar(0, 255, 255));
            }
        }
    }
    if (this->Winghat_stat == 0)
    {
        LOG_IF(INFO, this->switch_INFO == ON) << "selectWinghat failed";
        return 0;
    }
    else if (this->Winghat_stat == 1)
    {
        LOG_IF(INFO, this->switch_INFO == ON) << "selectWinghat successful";
        return 1;
    }
    else if (this->Wing_stat != 1)
    {
        // 假如Wing_stat不为1，已经确定的Wing不为一个，无法用于判断Winghat
        this->winghat_idx.clear();
        LOG_IF(INFO, this->switch_INFO == ON) << "selectWinghat failed, lack Wing for select";
        return 0;
    }
    else if (this->Winghat_stat == 2)
    {
        // 一直筛选直到只剩下一个Winghat
        while (winghat_idx.size() > 1)
        {
            // 假如说0号Winghat到扇叶下半部分的距离小于1号，删除1号，反之删除0号，这里使用距离的平方，省去开方运算，结果等价
            if (CalDistSquare(cv::minAreaRect(this->wing_contours[this->wing_idx[0]]).center, cv::minAreaRect(this->R_contours[this->winghat_idx[0]]).center) > CalDistSquare(cv::minAreaRect(this->wing_contours[this->wing_idx[0]]).center, cv::minAreaRect(this->R_contours[this->winghat_idx[1]]).center))
            {
                this->winghat_idx.pop_front();
            }
            else
            {
                // 不知道为什么，erase方法有一些问题，适用笨方法，删除第二个
                int temp = this->winghat_idx[0];
                this->winghat_idx.pop_front();
                this->winghat_idx.pop_front();
                this->winghat_idx.emplace_front(temp);
            }
        }
        // 根据当前idx数量输出状态
        if (winghat_idx.size() == 1)
        {
            LOG_IF(INFO, this->switch_INFO == ON) << "selectWinghat successful";
            return 1;
        }
        else if (winghat_idx.size() == 0)
        {
            LOG_IF(INFO, this->switch_INFO == ON) << "selectWinghat failed, no Winghat left";
            this->Winghat_stat = -1;
            return 0;
        }
        else if (winghat_idx.size() > 1)
        {
            LOG_IF(INFO, this->switch_INFO == ON) << "selectWinghat failed, too many Winghat left";
            this->Winghat_stat = -1;
            return 0;
        }
    }
    LOG_IF(INFO, this->switch_INFO == ON) << "selectWinghat failed";
    return 0;
}

int WMIdentify::getArmorCenterFloodfill()
{
    // 此方法当前废弃
    this->wing_idx.clear();
    cv::Mat src = this->binary.clone();
    // 进行图形学操作，最后新添加的膨胀后腐蚀可以让灯带连在一起
    cv::dilate(src, src, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(this->gp->dialte1, this->gp->dialte1)));
    cv::dilate(src, src, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9)));
    cv::Mat analysis;
    // 二值化，漫水处理之前必须要进行，生成地道二值化图
    cv::threshold(src, analysis, 0, 255, cv::THRESH_BINARY_INV);
    cv::morphologyEx(analysis, analysis, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10)));
#ifdef DEBUGMODE
    cv::imshow("after threshold", analysis);
#endif // DEBUGMODE
    // 漫水处理
    cv::floodFill(analysis, cv::Point(0, 0), cv::Scalar(0));
#ifdef DEBUGMODE
    cv::imshow("after floodFill", analysis);
#endif // DEBUGMODE
    // 闭操作，膨胀再腐蚀，使得非击打叶片变为一个大的整体
    // NOTE: 之后调试若有噪点，可以在闭操作之后加入开操作去除噪点
    // cv::morphologyEx(analysis, analysis, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(30, 30)));
    cv::morphologyEx(analysis, analysis, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(20, 20)));
#ifdef DEBUGMODE
    cv::imshow("after morphologyEx", analysis);
#endif // DEBUGMODE
    // 获取轮廓
    findContours(analysis, this->floodfill_contours, this->floodfill_hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // 遍历轮廓，如果是装甲板，将索引压入队列
    for (int i = 0; i < this->floodfill_contours.size(); i++)
    {
        if (IsValidArmorFloodfill(this->floodfill_contours[i], *gp))
        {
            this->wing_idx.emplace_back(i);
            cv::circle(img, cv::minAreaRect(floodfill_contours[0]).center, 10, cv::Scalar(0, 0, 255), -1);
            LOG_IF(INFO, this->switch_INFO) << "getArmorCenterFloodfill Successful";
            return 1;
        }
    }
    LOG_IF(ERROR, this->switch_ERROR) << "Failed to get center idx by getValidWingArmorFloodfill";
    return 0;
}

int WMIdentify::getWingCenterapproxPolyDP()
{
    // 角点检测方法，暂时没有采用
    return 1;
}

int WMIdentify::getVaildR()
{
    this->R_idx.clear();
    // 遍历全部的轮廓，通过轮廓特征判断是否是R
    for (int i = 0; i < this->R_contours.size(); i++)
    {
        if (IsVaildR(this->R_contours[i], *gp))
        {
            this->R_idx.emplace_back(i);
        }
    }
    // 通过当前找到的符合的R的数量，输出当前状态
    if (this->R_idx.size() == 0)
    {
        LOG_IF(INFO, this->switch_INFO) << "getVaildR Get No R";
        this->R_stat = 0;
    }
    else if (this->R_idx.size() == 1)
    {
        LOG_IF(INFO, this->switch_INFO) << "getVaildR Get One R";
        this->R_stat = 1;
    }
    else if (this->R_idx.size() > 1)
    {
        LOG_IF(INFO, this->switch_INFO) << "getVaildR Get Many R:" << this->R_idx.size();
        this->R_stat = 2;
    }
    return this->R_stat;
}

int WMIdentify::selectR()
{
    // 假如没有R，识别失败
    if (this->R_stat == 0)
    {
        LOG_IF(INFO, this->switch_INFO == ON) << "selectR failed";
        return 0;
    }
    // 假如只有一个R，没有选择的必要，直接成功
    else if (this->R_stat == 1)
    {
        LOG_IF(INFO, this->switch_INFO == ON) << "selectR successful";
        // 如果UI开关打开
        if (this->gp->switch_UI == ON)
        {
            // 遍历idx里面每一个索引(其实只有一个，保证结构一致性，不更改)
            for (int i = 0; i < R_idx.size(); i++)
            {
                // 如果轮廓显示开关打开，画出当前找到的R的轮廓
                if (this->gp->switch_UI_contours == ON)
                    cv::drawContours(this->img, this->R_contours, R_idx[i], cv::Scalar(0, 0, 255), 3);
                // 如果面积显示开关打开，显示出当前找到的R的面积、紧致度、圆度
                if (this->gp->switch_UI_areas == ON)
                {
                    double s_area = cv::contourArea(R_contours[R_idx[i]]);
                    cv::putText(this->img, std::to_string(s_area), cv::minAreaRect(this->R_contours[this->R_idx[i]]).center, 1, 1, cv::Scalar(255, 255, 0));
                    double girth = cv::arcLength(R_contours[R_idx[i]], true);
                    double compactness = (girth * girth) / s_area;
                    double circularity = (4 * Pi * s_area) / (girth * girth);
                    cv::putText(this->img, std::to_string(compactness), cv::minAreaRect(this->R_contours[this->R_idx[i]]).center + cv::Point2f(0, 20), 1, 1, cv::Scalar(255, 255, 0));
                    cv::putText(this->img, std::to_string(circularity), cv::minAreaRect(this->R_contours[this->R_idx[i]]).center + cv::Point2f(0, 40), 1, 1, cv::Scalar(255, 255, 0));
                }
            }
        }
        return 1;
    }
    // 假如有很多R，开始筛选
    else if (this->R_stat == 2)
    {
        // 如果UI开关打开
        if (this->gp->switch_UI == ON)
        {
            // 遍历idx里面每一个索引
            for (int i = 0; i < R_idx.size(); i++)
            {
                // 如果轮廓显示开关打开，画出当前找到的R的轮廓
                if (this->gp->switch_UI_contours == ON)
                    cv::drawContours(this->img, this->R_contours, R_idx[i], cv::Scalar(0, 0, 255), 3);
                // 如果面积显示开关打开，显示出当前找到的R的面积、紧致度、圆度
                if (this->gp->switch_UI_areas == ON)
                {
                    double s_area = cv::contourArea(R_contours[R_idx[i]]);
                    cv::putText(this->img, std::to_string(s_area), cv::minAreaRect(this->R_contours[this->R_idx[i]]).center, 1, 1, cv::Scalar(255, 255, 0));
                    double girth = cv::arcLength(R_contours[R_idx[i]], true);
                    double compactness = (girth * girth) / s_area;
                    double circularity = (4 * Pi * s_area) / (girth * girth);
                    cv::putText(this->img, std::to_string(compactness), cv::minAreaRect(this->R_contours[this->R_idx[i]]).center + cv::Point2f(0, 20), 1, 1, cv::Scalar(255, 255, 0));
                    cv::putText(this->img, std::to_string(circularity), cv::minAreaRect(this->R_contours[this->R_idx[i]]).center + cv::Point2f(0, 40), 1, 1, cv::Scalar(255, 255, 0));
                }
            }
        }
        // 遍历全部可能是R的索引，判断是否在中心区域，假如是，认为它是R，让R_idx首位是它，结束筛选
        // for (int i = 0; i < this->R_idx.size(); i++)
        // {
        //     if (cv::minAreaRect(this->R_contours[this->R_idx[i]]).center.x > this->img.cols * this->gp->R_roi_xl && cv::minAreaRect(R_contours[R_idx[i]]).center.x < this->img.cols * this->gp->R_roi_xr && cv::minAreaRect(R_contours[R_idx[i]]).center.y > this->img.rows * this->gp->R_roi_yb && cv::minAreaRect(R_contours[R_idx[i]]).center.y < this->img.rows * this->gp->R_roi_yt)
        //     {
        //         this->R_idx[0] = this->R_idx[i];
        //         LOG_IF(INFO, this->switch_INFO == ON) << "selectR successful";
        //     }
        // }
        std::sort(R_idx.begin(), R_idx.end(), [this](int &a, int &b)
                  { return CalDistSquare(cv::minAreaRect(R_contours[a]).center, R_emitate) < CalDistSquare(cv::minAreaRect(R_contours[b]).center, R_emitate); });
        if (this->R_idx.size() == 1)
        {
            LOG_IF(INFO, this->switch_INFO == ON) << "selectR successful";
            return 1;
        }
        else if (this->R_idx.size() == 0)
        {
            LOG_IF(INFO, this->switch_INFO == ON) << "selectR failed, no R left";
            return 0;
        }
        else if (this->R_idx.size() > 1)
        {
            LOG_IF(INFO, this->switch_INFO == ON) << "selectR failed, too many R left";
            return 0;
        }
    }
    LOG_IF(INFO, this->switch_INFO == ON) << "selectR failed";
    return 0;
}

void WMIdentify::updateList(double time)
{
#ifndef USEWMNET
    list_stat = 1; //<! 队列更新状态，为了保证每个队列中数据数量均同步且对应，1为没有缺失数据，0为缺失数据
    if (this->wing_center_list.size() >= 2)
    {
        this->wing_center_list.pop_front();
    }
    // 如果寻找装甲板中心点确实找到东西了，列表不为空，则把找到的装甲板中心点坐标压进队列中，否则输出日志，数据不足，同时判断当前识别装甲板模式，以通过不同的方式获取中心点坐标
    // winghat的方法时，winghat_idx与R_contours对应，详情间之前步骤注释
    if (this->winghat_idx.size() > 0 && this->R_contours.size() > 0 && this->get_armor_mode == HIERARCHY)
    {
        this->wing_center_list.emplace_back(cv::minAreaRect(this->R_contours[this->winghat_idx[0]]).center);
    }
    else if (this->wing_idx.size() > 0 && this->floodfill_contours.size() > 0 && this->get_armor_mode == FLOODFILL)
    {
        this->wing_center_list.emplace_back(cv::minAreaRect(this->floodfill_contours[this->wing_idx[0]]).center);
    }
    else
    {
        LOG_IF(ERROR, this->switch_ERROR) << "failed to emplace_back wing_center_list, lack data";
        list_stat = 0;
    }
#ifdef DEBUGMODE
    // 如果获取点信息正常，绘制在img上
    if (this->wing_center_list.size() > 0 && list_stat == 1)
        cv::circle(this->img, this->wing_center_list[this->wing_center_list.size() - 1], 10, cv::Scalar(0, 0, 255), -1);
#endif // DEBUGMODE
    if (this->R_center_list.size() >= 2)
    {
        this->R_center_list.pop_front();
    }
    // 如果寻找R中心点确实找到东西了，列表不为空，则把找到的R中心点坐标压进队列中，否则输出日志，数据不足
    if (this->R_idx.size() > 0 && this->R_contours.size() > 0 && list_stat == 1)
    {
        this->R_center_list.emplace_back(cv::minAreaRect(this->R_contours[this->R_idx[0]]).center);
    }
    else
    {
        LOG_IF(ERROR, this->switch_ERROR) << "failed to emplace_back R_center_list, lack data";
        if (list_stat == 1)
        {
            // 此时armor已经输入了数据，但是R无法输入数据，所以要把armor的数据吐出来，保证数据同步
            this->wing_center_list.pop_back();
        }
        list_stat = 0;
    }
#ifdef DEBUGMODE
    // 如果获取点信息正常，绘制在img上
    if (this->R_center_list.size() > 0 && list_stat == 1)
        cv::circle(this->img, this->R_center_list[this->R_center_list.size() - 1], 10, cv::Scalar(0, 255, 255), -1);
    if (this->get_armor_mode == HIERARCHY)
    {
        cv::imshow("result by hierarchy", img);
    }
    else if (this->get_armor_mode == FLOODFILL)
    {
        cv::imshow("result by floodfill", img);
    }
#endif // DEBUGMODE
#else
#ifdef DEBUGMODE
    if (this->R_center_list.size() > 0 && list_stat == 1)
        cv::circle(this->img, this->R_center_list[this->R_center_list.size() - 1], 10, cv::Scalar(0, 255, 255), -1);
    if (this->wing_center_list.size() > 0 && list_stat == 1)
        cv::circle(this->img, this->wing_center_list[this->wing_center_list.size() - 1], 10, cv::Scalar(0, 0, 255), -1);
#endif
#endif
    // 更新时间队列
    if (this->time_list.size() >= this->gp->list_size && gp->gap % gp->gap_control == 0)
    {
        this->time_list.pop_front();
    }
    if (list_stat == 1 && gp->gap % gp->gap_control == 0)
    {
        this->time_list.emplace_back(time);
    }
    // 更新角度队列
    if (this->angle_list.size() >= this->gp->list_size && gp->gap % gp->gap_control == 0)
    {
        this->angle_list.pop_front();
    }
    if (list_stat == 1 && gp->gap % gp->gap_control == 0)
    {
        this->angle_list.emplace_back(CalAngle(this->wing_center_list.back(), this->R_center_list.back()));
    }

    // 更新yaw队列
    if (this->R_yaw_list.size() >= this->gp->list_size)
    {
        this->R_yaw_list.pop_front();
    }
    if (list_stat == 1)
    {
        float u = R_center_list[R_center_list.size() - 1].x;
        float v = R_center_list[R_center_list.size() - 1].y;
        float tan_x = (u - (double)this->img.cols / 2) / this->gp->fx;
        // std::cout<<tan_x<<std::endl;
        float yaw = 0;
        if (abs(tan_x) > 0.0001)
            yaw = atan(tan_x);
        this->R_yaw_list.emplace_back(yaw);
    }
    // 更新角速度队列
    if (this->angle_velocity_list.size() >= this->gp->list_size && gp->gap % gp->gap_control == 0)
    {
        this->angle_velocity_list.pop_front();
    }
    if (angle_list.size() > 1 && time_list.size() > 1 && list_stat == 1 && gp->gap % gp->gap_control == 0 )
    {
        // 计算角度变化量
        double dangle = this->angle_list[this->angle_list.size() - 1] - this->angle_list[this->angle_list.size() - 2];
        // 防止数据跳变
        dangle += (abs(dangle) > Pi) ? 2 * Pi * (-dangle / abs(dangle)) : 0;
        // 计算时间变化量
        double dtime = (this->time_list[this->time_list.size() - 1] - this->time_list[this->time_list.size() - 2]);
        // 更新角速度队列,简单检验一下数据
        if (abs(dangle / dtime) > 2.2)
        {
            this->time_list.pop_front();
            this->angle_list.pop_front();
            gp->gap--;
        }
        else
        {
            this->angle_velocity_list.emplace_back(dangle / dtime);
#ifdef DEBUGHIT
            if (angle_velocity_list.size() >= 2)
            {

                cv::Point2f now_point(time_list[time_list.size() - 1], abs(angle_velocity_list[angle_velocity_list.size() - 1]));
                static cv::Point2f first_point(time_list.back(), abs(angle_velocity_list.back()));
                cv::Point2f last_point(time_list[time_list.size() - 2], abs(angle_velocity_list[angle_velocity_list.size() - 2]));
                now_point.x -= first_point.x;
                now_point.x *= 20;
                now_point.y = now_point.y * 100;
                last_point.x -= first_point.x;
                last_point.x *= 20;
                last_point.y = last_point.y * 100;
                cv::circle(this->data_img, now_point, 1, cv::Scalar(255, 255, 255));
            }

#endif
        }

        // 更新旋转方向
        // std::cout<<this->time_list.back()<<std::endl;
        // std::cout<<" "<<this->angle_velocity_list.back()<<std::endl;
        this->direction = 0;
        for (int i = 0; i < angle_velocity_list.size(); i++)
        {
            this->direction += this->angle_velocity_list[i];
        }
    }
    // 更新gap
    if (this->list_stat == 1)
    {
        gp->gap++;
    }
    // 输出日志，更新队列成功
    LOG_IF(INFO, this->switch_INFO == ON) << "updateList successful";
}

std::deque<double> WMIdentify::getTimeList()
{
    return this->time_list;
}

std::deque<double> WMIdentify::getAngleVelocityList()
{
    return this->angle_velocity_list;
}
double WMIdentify::getDirection()
{
    return this->direction;
}
double WMIdentify::getAngleList()
{
    return this->angle_list[angle_list.size() - 1];
}
cv::Point2d WMIdentify::getR_center()
{
    return this->R_center_list[R_center_list.size() - 1];
}
double WMIdentify::getRadius()
{
    return sqrt(CalDistSquare(this->R_center_list[R_center_list.size() - 1], this->wing_center_list[wing_center_list.size() - 1]));
}
double WMIdentify::getYaw()
{
    if (R_yaw_list.size() > 0)
        return this->R_yaw_list[R_yaw_list.size() - 1];
    else
        return 0;
}

int WMIdentify::getListStat()
{
    return this->list_stat;
}
cv::Mat WMIdentify::getImg0()
{
    return this->img_0;
}
cv::Mat WMIdentify::getData_img()
{
    return this->data_img;
}
void WMIdentify::ClearSpeed()
{
    this->angle_velocity_list.clear();
    this->time_list.clear();
}