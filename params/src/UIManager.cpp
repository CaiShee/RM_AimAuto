/**
 * @file WMIdentify_UI.cpp
 * @author 高宁 (3406402603@qq.com)
 * @brief 因为调试过程中的UI系统与本体过于无关，而且因为架构比较无脑，很占地方，所以将其与本身的WMIdentify.cpp分开
 * @version 0.1
 * @date 2023-02-20
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <UIManager.hpp>

UIManager::UIManager()
{
    this->page = 0;
    this->row = 0;
}

UIManager::~UIManager()
{
}

void UIManager::receive_pic(cv::Mat img)
{
    this->img = img;
}

void UIManager::windowsManager(GlobalParam &gp, int key, int &debug_t)
{
    tag = 0;
    if (key == 'w' || key == 'W')
    {
        if (row > 0)
            row--;
    }
    if (key == 's' || key == 'S')
    {
        if (row < 5)
            row++;
    }
    if (key == 'a' || key == 'A')
    {
        if (page > 0)
            page--;
    }
    if (key == 'd' || key == 'D')
    {
        if (page < 6)
            page++;
    }
    if (key == 'o' || key == 'O')
    {
        debug_t *= 2;
    }
    if (key == 'l' || key == 'L')
    {
        if (debug_t > 1)
            debug_t /= 2;
        else
            debug_t = 1;
    }
    if (key == 'u' || key == 'U')
    {
        tag = 1;
    }
    if (key == 'j' || key == 'J')
    {
        tag = 3;
    }
    if (key == 'i' || key == 'I')
    {
        tag = 2;
    }
    if (key == 'k' || key == 'K')
    {
        tag = 4;
    }
    cv::putText(this->img, "Page:" + std::to_string(page) + "  Left:Z  Right:C", cv::Point(20, 400), 2, 0.5, cv::Scalar(255, 255, 0));
    cv::putText(this->img, "Row:" + std::to_string(row) + "  Up:R  Down:F", cv::Point(20, 425), 2, 0.5, cv::Scalar(255, 255, 0));
    if (page == 0)
    {
        if (row == 0)
            cv::putText(this->img, "HMax:" + std::to_string(gp.hmax), cv::Point(20, 20), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "HMax:" + std::to_string(gp.hmax), cv::Point(20, 20), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 1)
            cv::putText(this->img, "HMin:" + std::to_string(gp.hmin), cv::Point(20, 45), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "HMin:" + std::to_string(gp.hmin), cv::Point(20, 45), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 2)
            cv::putText(this->img, "SMax:" + std::to_string(gp.smax), cv::Point(20, 70), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "SMax:" + std::to_string(gp.smax), cv::Point(20, 70), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 3)
            cv::putText(this->img, "SMin:" + std::to_string(gp.smin), cv::Point(20, 95), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "SMin:" + std::to_string(gp.smin), cv::Point(20, 95), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 4)
            cv::putText(this->img, "VMax:" + std::to_string(gp.vmax), cv::Point(20, 120), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "VMax:" + std::to_string(gp.vmax), cv::Point(20, 120), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 5)
            cv::putText(this->img, "VMin:" + std::to_string(gp.vmin), cv::Point(20, 145), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "VMin:" + std::to_string(gp.vmin), cv::Point(20, 145), 2, 0.5, cv::Scalar(0, 0, 255));
        if (tag == 1)
        {
            if (row == 0)
                gp.hmax += 1;
            if (row == 1)
                gp.hmin += 1;
            if (row == 2)
                gp.smax += 1;
            if (row == 3)
                gp.smin += 1;
            if (row == 4)
                gp.vmax += 1;
            if (row == 5)
                gp.vmin += 1;
        }
        else if (tag == 2)
        {
            if (row == 0)
                gp.hmax += 10;
            if (row == 1)
                gp.hmin += 10;
            if (row == 2)
                gp.smax += 10;
            if (row == 3)
                gp.smin += 10;
            if (row == 4)
                gp.vmax += 10;
            if (row == 5)
                gp.vmin += 10;
        }
        if (tag == 3)
        {
            if (row == 0)
                gp.hmax -= 1;
            if (row == 1)
                gp.hmin -= 1;
            if (row == 2)
                gp.smax -= 1;
            if (row == 3)
                gp.smin -= 1;
            if (row == 4)
                gp.vmax -= 1;
            if (row == 5)
                gp.vmin -= 1;
        }
        else if (tag == 4)
        {
            if (row == 0)
                gp.hmax -= 10;
            if (row == 1)
                gp.hmin -= 10;
            if (row == 2)
                gp.smax -= 10;
            if (row == 3)
                gp.smin -= 10;
            if (row == 4)
                gp.vmax -= 10;
            if (row == 5)
                gp.vmin -= 10;
        }
    }
    else if (page == 1)
    {
        if (row == 0)
            cv::putText(this->img, "Enemy HMax:" + std::to_string(gp.e_hmax), cv::Point(20, 20), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "Enemy HMax:" + std::to_string(gp.e_hmax), cv::Point(20, 20), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 1)
            cv::putText(this->img, "Enemy HMin:" + std::to_string(gp.e_hmin), cv::Point(20, 45), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "Enemy HMin:" + std::to_string(gp.e_hmin), cv::Point(20, 45), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 2)
            cv::putText(this->img, "Enemy SMax:" + std::to_string(gp.e_smax), cv::Point(20, 70), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "Enemy SMax:" + std::to_string(gp.e_smax), cv::Point(20, 70), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 3)
            cv::putText(this->img, "Enemy SMin:" + std::to_string(gp.e_smin), cv::Point(20, 95), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "Enemy SMin:" + std::to_string(gp.e_smin), cv::Point(20, 95), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 4)
            cv::putText(this->img, "Enemy VMax:" + std::to_string(gp.e_vmax), cv::Point(20, 120), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "Enemy VMax:" + std::to_string(gp.e_vmax), cv::Point(20, 120), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 5)
            cv::putText(this->img, "Enemy VMin:" + std::to_string(gp.e_vmin), cv::Point(20, 145), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "Enemy VMin:" + std::to_string(gp.e_vmin), cv::Point(20, 145), 2, 0.5, cv::Scalar(0, 0, 255));
        if (tag == 1)
        {
            if (row == 0)
                gp.e_hmax += 1;
            if (row == 1)
                gp.e_hmin += 1;
            if (row == 2)
                gp.e_smax += 1;
            if (row == 3)
                gp.e_smin += 1;
            if (row == 4)
                gp.e_vmax += 1;
            if (row == 5)
                gp.e_vmin += 1;
        }
        else if (tag == 2)
        {
            if (row == 0)
                gp.e_hmax += 10;
            if (row == 1)
                gp.e_hmin += 10;
            if (row == 2)
                gp.e_smax += 10;
            if (row == 3)
                gp.e_smin += 10;
            if (row == 4)
                gp.e_vmax += 10;
            if (row == 5)
                gp.e_vmin += 10;
        }
        if (tag == 3)
        {
            if (row == 0)
                gp.e_hmax -= 1;
            if (row == 1)
                gp.e_hmin -= 1;
            if (row == 2)
                gp.e_smax -= 1;
            if (row == 3)
                gp.e_smin -= 1;
            if (row == 4)
                gp.e_vmax -= 1;
            if (row == 5)
                gp.e_vmin -= 1;
        }
        else if (tag == 4)
        {
            if (row == 0)
                gp.e_hmax -= 10;
            if (row == 1)
                gp.e_hmin -= 10;
            if (row == 2)
                gp.e_smax -= 10;
            if (row == 3)
                gp.e_smin -= 10;
            if (row == 4)
                gp.e_vmax -= 10;
            if (row == 5)
                gp.e_vmin -= 10;
        }
    }
    else if (page == 2)
    {
        if (row == 0)
            cv::putText(this->img, "s_R_min:" + std::to_string(gp.s_R_min), cv::Point(20, 20), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "s_R_min:" + std::to_string(gp.s_R_min), cv::Point(20, 20), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 1)
            cv::putText(this->img, "s_R_max:" + std::to_string(gp.s_R_max), cv::Point(20, 45), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "s_R_max:" + std::to_string(gp.s_R_max), cv::Point(20, 45), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 2)
            cv::putText(this->img, "R_circularity_min:" + std::to_string(gp.R_circularity_min), cv::Point(20, 70), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "R_circularity_min:" + std::to_string(gp.R_circularity_min), cv::Point(20, 70), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 3)
            cv::putText(this->img, "R_circularity_max:" + std::to_string(gp.R_circularity_max), cv::Point(20, 95), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "R_circularity_max:" + std::to_string(gp.R_circularity_max), cv::Point(20, 95), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 4)
            cv::putText(this->img, "R_compactness_min:" + std::to_string(gp.R_compactness_min), cv::Point(20, 120), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "R_compactness_min:" + std::to_string(gp.R_compactness_min), cv::Point(20, 120), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 5)
            cv::putText(this->img, "R_compactness_max:" + std::to_string(gp.R_compactness_max), cv::Point(20, 145), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "R_compactness_max:" + std::to_string(gp.R_compactness_max), cv::Point(20, 145), 2, 0.5, cv::Scalar(0, 0, 255));
        if (tag == 1)
        {
            if (row == 0)
                gp.s_R_min += 10;
            if (row == 1)
                gp.s_R_max += 10;
            if (row == 2)
                gp.R_circularity_min += 0.01;
            if (row == 3)
                gp.R_circularity_max += 0.01;
            if (row == 4)
                gp.R_compactness_min += 1;
            if (row == 5)
                gp.R_compactness_max += 1;
        }
        else if (tag == 2)
        {
            if (row == 0)
                gp.s_R_min += 100;
            if (row == 1)
                gp.s_R_max += 100;
            if (row == 2)
                gp.R_circularity_min += 0.1;
            if (row == 3)
                gp.R_circularity_max += 0.1;
            if (row == 4)
                gp.R_compactness_min += 10;
            if (row == 5)
                gp.R_compactness_max += 10;
        }
        if (tag == 3)
        {
            if (row == 0)
                gp.s_R_min -= 10;
            if (row == 1)
                gp.s_R_max -= 10;
            if (row == 2)
                gp.R_circularity_min -= 0.01;
            if (row == 3)
                gp.R_circularity_max -= 0.01;
            if (row == 4)
                gp.R_compactness_min -= 1;
            if (row == 5)
                gp.R_compactness_max -= 1;
        }
        else if (tag == 4)
        {
            if (row == 0)
                gp.s_R_min -= 100;
            if (row == 1)
                gp.s_R_max -= 100;
            if (row == 2)
                gp.R_circularity_min -= 0.1;
            if (row == 3)
                gp.R_circularity_max -= 0.1;
            if (row == 4)
                gp.R_compactness_min -= 10;
            if (row == 5)
                gp.R_compactness_max -= 10;
        }
    }
    else if (page == 3)
    {
        if (row == 0)
            cv::putText(this->img, "s_wing_min:" + std::to_string(gp.s_wing_min), cv::Point(20, 20), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "s_wing_min:" + std::to_string(gp.s_wing_min), cv::Point(20, 20), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 1)
            cv::putText(this->img, "s_wing_max:" + std::to_string(gp.s_wing_max), cv::Point(20, 45), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "s_wing_max:" + std::to_string(gp.s_wing_max), cv::Point(20, 45), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 2)
            cv::putText(this->img, "wing_ratio_min:" + std::to_string(gp.wing_ratio_min), cv::Point(20, 70), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "wing_ratio_min:" + std::to_string(gp.wing_ratio_min), cv::Point(20, 70), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 3)
            cv::putText(this->img, "wing_ratio_max:" + std::to_string(gp.wing_ratio_max), cv::Point(20, 95), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "wing_ratio_max:" + std::to_string(gp.wing_ratio_max), cv::Point(20, 95), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 4)
            cv::putText(this->img, "s_wing_ratio_min:" + std::to_string(gp.s_wing_ratio_min), cv::Point(20, 120), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "s_wing_ratio_min:" + std::to_string(gp.s_wing_ratio_min), cv::Point(20, 120), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 5)
            cv::putText(this->img, "s_wing_ratio_max:" + std::to_string(gp.s_wing_ratio_max), cv::Point(20, 145), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "s_wing_ratio_max:" + std::to_string(gp.s_wing_ratio_max), cv::Point(20, 145), 2, 0.5, cv::Scalar(0, 0, 255));
        if (tag == 1)
        {
            if (row == 0)
                gp.s_wing_min += 10;
            if (row == 1)
                gp.s_wing_max += 10;
            if (row == 2)
                gp.wing_ratio_min += 0.01;
            if (row == 3)
                gp.wing_ratio_max += 0.01;
            if (row == 4)
                gp.s_wing_ratio_min += 0.01;
            if (row == 5)
                gp.s_wing_ratio_max += 0.01;
        }
        else if (tag == 2)
        {
            if (row == 0)
                gp.s_wing_min += 100;
            if (row == 1)
                gp.s_wing_max += 100;
            if (row == 2)
                gp.wing_ratio_min += 0.1;
            if (row == 3)
                gp.wing_ratio_max += 0.1;
            if (row == 4)
                gp.s_wing_ratio_min += 0.1;
            if (row == 5)
                gp.s_wing_ratio_max += 0.1;
        }
        if (tag == 3)
        {
            if (row == 0)
                gp.s_wing_min -= 10;
            if (row == 1)
                gp.s_wing_max -= 10;
            if (row == 2)
                gp.wing_ratio_min -= 0.01;
            if (row == 3)
                gp.wing_ratio_max -= 0.01;
            if (row == 4)
                gp.s_wing_ratio_min -= 0.01;
            if (row == 5)
                gp.s_wing_ratio_max -= 0.01;
        }
        else if (tag == 4)
        {
            if (row == 0)
                gp.s_wing_min -= 100;
            if (row == 1)
                gp.s_wing_max -= 100;
            if (row == 2)
                gp.wing_ratio_min -= 0.1;
            if (row == 3)
                gp.wing_ratio_max -= 0.1;
            if (row == 4)
                gp.s_wing_ratio_min -= 0.1;
            if (row == 5)
                gp.s_wing_ratio_max -= 0.1;
        }
    }
    else if (page == 4)
    {
        if (row == 0)
            cv::putText(this->img, "s_winghat_min:" + std::to_string(gp.s_winghat_min), cv::Point(20, 20), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "s_winghat_min:" + std::to_string(gp.s_winghat_min), cv::Point(20, 20), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 1)
            cv::putText(this->img, "s_winghat_max:" + std::to_string(gp.s_winghat_max), cv::Point(20, 45), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "s_winghat_max:" + std::to_string(gp.s_winghat_max), cv::Point(20, 45), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 2)
            cv::putText(this->img, "winghat_ratio_min:" + std::to_string(gp.winghat_ratio_min), cv::Point(20, 70), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "winghat_ratio_min:" + std::to_string(gp.winghat_ratio_min), cv::Point(20, 70), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 3)
            cv::putText(this->img, "winghat_ratio_max:" + std::to_string(gp.winghat_ratio_max), cv::Point(20, 95), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "winghat_ratio_max:" + std::to_string(gp.winghat_ratio_max), cv::Point(20, 95), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 4)
            cv::putText(this->img, "s_winghat_ratio_min:" + std::to_string(gp.s_winghat_ratio_min), cv::Point(20, 120), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "s_winghat_ratio_min:" + std::to_string(gp.s_winghat_ratio_min), cv::Point(20, 120), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 5)
            cv::putText(this->img, "s_winghat_ratio_max:" + std::to_string(gp.s_winghat_ratio_max), cv::Point(20, 145), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "s_winghat_ratio_max:" + std::to_string(gp.s_winghat_ratio_max), cv::Point(20, 145), 2, 0.5, cv::Scalar(0, 0, 255));
        if (tag == 1)
        {
            if (row == 0)
                gp.s_winghat_min += 10;
            if (row == 1)
                gp.s_winghat_max += 10;
            if (row == 2)
                gp.winghat_ratio_min += 0.01;
            if (row == 3)
                gp.winghat_ratio_max += 0.01;
            if (row == 4)
                gp.s_winghat_ratio_min += 0.01;
            if (row == 5)
                gp.s_winghat_ratio_max += 0.01;
        }
        else if (tag == 2)
        {
            if (row == 0)
                gp.s_winghat_min += 100;
            if (row == 1)
                gp.s_winghat_max += 100;
            if (row == 2)
                gp.winghat_ratio_min += 0.1;
            if (row == 3)
                gp.winghat_ratio_max += 0.1;
            if (row == 4)
                gp.s_winghat_ratio_min += 0.1;
            if (row == 5)
                gp.s_winghat_ratio_max += 0.1;
        }
        if (tag == 3)
        {
            if (row == 0)
                gp.s_winghat_min -= 10;
            if (row == 1)
                gp.s_winghat_max -= 10;
            if (row == 2)
                gp.winghat_ratio_min -= 0.01;
            if (row == 3)
                gp.winghat_ratio_max -= 0.01;
            if (row == 4)
                gp.s_winghat_ratio_min -= 0.01;
            if (row == 5)
                gp.s_winghat_ratio_max -= 0.01;
        }
        else if (tag == 4)
        {
            if (row == 0)
                gp.s_winghat_min -= 100;
            if (row == 1)
                gp.s_winghat_max -= 100;
            if (row == 2)
                gp.winghat_ratio_min -= 0.1;
            if (row == 3)
                gp.winghat_ratio_max -= 0.1;
            if (row == 4)
                gp.s_winghat_ratio_min -= 0.1;
            if (row == 5)
                gp.s_winghat_ratio_max -= 0.1;
        }
    }
    else if (page == 5)
    {
        if (row == 0)
            cv::putText(this->img, "dialte1:" + std::to_string(gp.dialte1), cv::Point(20, 20), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "dialte1:" + std::to_string(gp.dialte1), cv::Point(20, 20), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 1)
            cv::putText(this->img, "dialte2:" + std::to_string(gp.dialte2), cv::Point(20, 45), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "dialte2:" + std::to_string(gp.dialte2), cv::Point(20, 45), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 2)
            cv::putText(this->img, "dialte3:" + std::to_string(gp.dialte3), cv::Point(20, 70), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "dialte3:" + std::to_string(gp.dialte3), cv::Point(20, 70), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 3)
            cv::putText(this->img, "switch_UI_contours:" + std::to_string(gp.switch_UI_contours), cv::Point(20, 95), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "switch_UI_contours:" + std::to_string(gp.switch_UI_contours), cv::Point(20, 95), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 4)
            cv::putText(this->img, "switch_UI_areas:" + std::to_string(gp.switch_UI_areas), cv::Point(20, 120), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "switch_UI_areas:" + std::to_string(gp.switch_UI_areas), cv::Point(20, 120), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 5)
            cv::putText(this->img, "switch_UI:" + std::to_string(gp.switch_UI), cv::Point(20, 145), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "switch_UI:" + std::to_string(gp.switch_UI), cv::Point(20, 145), 2, 0.5, cv::Scalar(0, 0, 255));
        if (tag == 1)
        {
            if (row == 0)
                gp.dialte1 += 1;
            if (row == 1)
                gp.dialte2 += 1;
            if (row == 2)
                gp.dialte3 += 1;
            if (row == 3)
                gp.switch_UI_contours = 1 - gp.switch_UI_contours;
            if (row == 4)
                gp.switch_UI_areas = 1 - gp.switch_UI_areas;
            if (row == 5)
                gp.switch_UI = 1 - gp.switch_UI;
        }
        if (tag == 3)
        {
            if (row == 0 && gp.dialte1 > 1)
                gp.dialte1 -= 1;
            if (row == 1 && gp.dialte2 > 1)
                gp.dialte2 -= 1;
            if (row == 2 && gp.dialte3 > 1)
                gp.dialte3 -= 1;
            if (row == 3)
                gp.switch_UI_contours = 1 - gp.switch_UI_contours;
            if (row == 4)
                gp.switch_UI_areas = 1 - gp.switch_UI_areas;
            if (row == 5)
                gp.switch_UI = 1 - gp.switch_UI;
        }
    }
    else if (page == 6)
    {
        if (row == 0)
            cv::putText(this->img, "armor_exp_time:" + std::to_string(gp.armor_exp_time), cv::Point(20, 20), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "armor_exp_time:" + std::to_string(gp.armor_exp_time), cv::Point(20, 20), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 1)
            cv::putText(this->img, "energy_exp_time:" + std::to_string(gp.energy_exp_time), cv::Point(20, 45), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "energy_exp_time:" + std::to_string(gp.energy_exp_time), cv::Point(20, 45), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 2)
            cv::putText(this->img, "fake_yaw:" + std::to_string(gp.fake_yaw), cv::Point(20, 70), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "fake_yaw:" + std::to_string(gp.fake_yaw), cv::Point(20, 70), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 3)
            cv::putText(this->img, "armorStat:" + std::to_string(gp.armorStat), cv::Point(20, 95), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "armorStat:" + std::to_string(gp.armorStat), cv::Point(20, 95), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 4)
            cv::putText(this->img, "attack_mode:" + std::to_string(gp.attack_mode), cv::Point(20, 120), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "attack_mode:" + std::to_string(gp.attack_mode), cv::Point(20, 120), 2, 0.5, cv::Scalar(0, 0, 255));
        if (row == 5)
            cv::putText(this->img, "color:" + std::to_string(gp.color), cv::Point(20, 145), 2, 0.5, cv::Scalar(255, 0, 0));
        else
            cv::putText(this->img, "color:" + std::to_string(gp.color), cv::Point(20, 145), 2, 0.5, cv::Scalar(0, 0, 255));
        if (tag == 1)
        {
            if (row == 0)
                gp.armor_exp_time += 100;
            if (row == 1)
                gp.energy_exp_time += 100;
            if (row == 2)
                gp.fake_yaw += 0.01;
            if (row == 3)
                gp.armorStat += 1;
            if (row == 4)
                gp.attack_mode = 1 - gp.attack_mode;
            if (row == 5)
                gp.color = 1 - gp.color;
        }
        else if (tag == 2)
        {
            if (row == 0)
                gp.armor_exp_time += 1000;
            if (row == 1)
                gp.energy_exp_time += 1000;
            if (row == 2)
                gp.fake_yaw += 0.1;
            if (row == 3)
                gp.armorStat += 2;
            if (row == 4)
                gp.attack_mode = 1 - gp.attack_mode;
            if (row == 5)
                gp.color = 1 - gp.color;
        }
        if (tag == 3)
        {
            if (row == 0)
                gp.armor_exp_time -= 100;
            if (row == 1)
                gp.energy_exp_time -= 100;
            if (row == 2)
                gp.fake_yaw -= 0.01;
            if (row == 3)
                gp.armorStat -= 1;
            if (row == 4)
                gp.attack_mode = 1 - gp.attack_mode;
            if (row == 5)
                gp.color = 1 - gp.color;
        }
        else if (tag == 4)
        {
            if (row == 0)
                gp.armor_exp_time -= 1000;
            if (row == 1)
                gp.energy_exp_time -= 1000;
            if (row == 2)
                gp.fake_yaw -= 0.1;
            if (row == 3)
                gp.armorStat -= 2;
            if (row == 4)
                gp.attack_mode = 1 - gp.attack_mode;
            if (row == 5)
                gp.color = 1 - gp.color;
        }
    }
}