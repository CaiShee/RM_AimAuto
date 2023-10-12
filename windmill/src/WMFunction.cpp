#include "WMFunction.hpp"

bool IsVaildWingHalf(std::vector<cv::Point> contour, GlobalParam &gp)
{
    double s_area = cv::contourArea(contour);
    // if (s_area < 150)
    //     return false;
    if (s_area < gp.s_wing_min || s_area > gp.s_wing_max)
    {
        LOG_IF(INFO, gp.switch_INFO) << "IsValidwing Successful But Not Wing,s_wing wrong:" << s_area;
        return false;
    }
    // cv::RotatedRect armor_rect = minAreaRect(contour);
    // cv::Size2f armor_size = armor_rect.size;
    // float length = armor_size.height > armor_size.width ? armor_size.height : armor_size.width; // 将矩形的长边设置为长
    // float width = armor_size.height < armor_size.width ? armor_size.height : armor_size.width;  // 将矩形的短边设置为宽
    // float lw_ratio = length / width;
    // float s_ratio = s_area / armor_size.area();
    // if (lw_ratio < gp.wing_ratio_min || lw_ratio > gp.wing_ratio_max)
    // {
    //     LOG_IF(INFO, gp.switch_INFO) << "IsValidwing Successful But Not Wing,lw_ratio wrong:" << lw_ratio << " area:" << s_area;
    //     return false;
    // }
    // if (s_ratio < gp.s_wing_ratio_min || s_ratio > gp.s_wing_ratio_max)
    // {
    //     LOG_IF(INFO, gp.switch_INFO) << "IsValidwing Successful But Not Wing,s_armor_ratio wrong:" << s_ratio << " area:" << s_area;
    //     return false;
    // }
    LOG_IF(INFO, gp.switch_INFO) << "IsValidwing Successful:" << s_area;
    return true;
}

bool IsValidArmorFloodfill(std::vector<cv::Point> contour, GlobalParam &gp)
{
    double s_area = cv::contourArea(contour);
    if (s_area < gp.s_armor_min_floodfill || s_area > gp.s_armor_max_floodfill)
    {
        LOG_IF(INFO, gp.switch_INFO) << "IsValidArmorFloodfill Successful But Not Armor,s_armor wrong:" << s_area;
        return false;
    }
    cv::RotatedRect armor_rect = minAreaRect(contour);
    cv::Size2f armor_size = armor_rect.size;
    float length = armor_size.height > armor_size.width ? armor_size.height : armor_size.width; // 将矩形的长边设置为长
    float width = armor_size.height < armor_size.width ? armor_size.height : armor_size.width;  // 将矩形的短边设置为宽
    float lw_ratio = length / width;
    float s_ratio = s_area / armor_size.area();
    if (lw_ratio < gp.armor_ratio_min_floodfill || lw_ratio > gp.armor_ratio_max_floodfill)
    {
        LOG_IF(INFO, gp.switch_INFO) << "IsValidArmorFloodfill Successful But Not Armor,lw_ratio wrong:" << lw_ratio;
        return false;
    }
    if (s_ratio < gp.s_armor_ratio_min_floodfill || s_ratio > gp.s_armor_ratio_max_floodfill)
    {
        LOG_IF(INFO, gp.switch_INFO) << "IsValidArmorFloodfill Successful But Not Armor,s_armor_ratio wrong:" << s_ratio;
        return false;
    }
    LOG_IF(INFO, gp.switch_INFO) << "IsValidArmorFloodfill Successful";
    return true;
}

bool IsVaildWingHat(std::vector<cv::Point> contour, GlobalParam &gp)
{
    double s_area = cv::contourArea(contour);
    // if (s_area < 150)
    //     return false;
    if (s_area < gp.s_winghat_min || s_area > gp.s_winghat_max)
    {
        LOG_IF(INFO, gp.switch_INFO) << "IsVaildWingHat Successful But Not WingHat,s_area wrong:" << s_area;
        return false;
    }
    cv::RotatedRect armor_rect = minAreaRect(contour);
    cv::Size2f armor_size = armor_rect.size;
    float length = armor_size.height > armor_size.width ? armor_size.height : armor_size.width; // 将矩形的长边设置为长
    float width = armor_size.height < armor_size.width ? armor_size.height : armor_size.width;  // 将矩形的短边设置为宽
    float lw_ratio = length / width;
    float s_ratio = s_area / armor_size.area();
    if (lw_ratio < gp.winghat_ratio_min || lw_ratio > gp.winghat_ratio_max)
    {
        LOG_IF(INFO, gp.switch_INFO) << "IsVaildWingHat Successful But Not WingHat,lw_ratio wrong:" << lw_ratio;
        return false;
    }
    if (s_ratio < gp.s_winghat_ratio_min || s_ratio > gp.s_winghat_ratio_max)
    {
        LOG_IF(INFO, gp.switch_INFO) << "IsVaildWingHat Successful But Not WingHat,s_ratio wrong:" << s_ratio;
        return false;
    }
    LOG_IF(INFO, gp.switch_INFO) << "IsVaildWingHat Successful:" << s_area;
    return true;
}

bool IsVaildR(std::vector<cv::Point> contour, GlobalParam &gp)
{
    double Pi = 3.1415926;
    double s_area = cv::contourArea(contour);
    double girth = cv::arcLength(contour, true);
    // if (s_area < 150)
    //     return false;
    if (s_area < gp.s_R_min || s_area > gp.s_R_max)
    {
        LOG_IF(INFO, gp.switch_INFO) << "IsVaildR Successful But Not R,s_area wrong:" << s_area;
        return false;
    }
    cv::RotatedRect R_rect = minAreaRect(contour);
    cv::Size2f R_size = R_rect.size;
    float length = R_size.height > R_size.width ? R_size.height : R_size.width; // 将矩形的长边设置为长
    float width = R_size.height < R_size.width ? R_size.height : R_size.width;  // 将矩形的短边设置为宽
    float lw_ratio = length / width;
    float s_ratio = s_area / R_size.area();
    if (lw_ratio < gp.R_ratio_min || lw_ratio > gp.R_ratio_max)
    {
        LOG_IF(INFO, gp.switch_INFO) << "IsVaildR Successful But Not R,lw_ratio wrong:" << lw_ratio << " and s_area is:" << s_area;
        return false;
    }
    if (s_ratio < gp.s_R_ratio_min || s_ratio > gp.s_R_ratio_max)
    {
        LOG_IF(INFO, gp.switch_INFO) << "IsVaildR Successful But Not R,s_ratio wrong:" << s_ratio << " and s_area is:" << s_area;
        return false;
    }
    double compactness = (girth * girth) / s_area;
    double circularity = (4 * Pi * s_area) / (girth * girth);
    if (circularity < gp.R_circularity_min || circularity > gp.R_circularity_max)
    {
        LOG_IF(INFO, gp.switch_INFO) << "IsVaildR Successful But Not R,circularity wrong:" << circularity << " and s_area is:" << s_area;
        return false;
    }
    if (compactness < gp.R_compactness_min || compactness > gp.R_compactness_max)
    {
        LOG_IF(INFO, gp.switch_INFO) << "IsVaildR Successful But Not R,compactness wrong:" << compactness << " and s_area is:" << s_area;
        return false;
    }
    LOG_IF(INFO, gp.switch_INFO) << "IsVaildR Successful,compactness:" << compactness << " circularity:" << circularity << " s_area:" << s_area;
    return true;
}

double CalAngle(cv::Point p1, cv::Point center)
{
    double Pi = 3.1415926;
    if(abs(p1.x-center.x)<0.01)
    {
        if(p1.y>center.y)
            return -Pi/2 ;
        else
            return Pi/2;
    }
   
    cv::Point2f dp = p1 - center;
    // 因为图像中y轴向下，所以需要翻转
    double angle = atan2(-dp.y, dp.x);
    if (angle >= 0)
        return angle;
    else
        return angle + 2 * Pi;
    return 0;
}

double CalDistSquare(cv::Point2f p1, cv::Point2f p2)
{
    return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}