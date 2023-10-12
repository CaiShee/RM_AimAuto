/**
 * @file globalParam.hpp
 * @author axi404 (3406402603@qq.com)
 * @brief 参数列表文件，由结构体与枚举组成，暂时仅包括打符识别需要的参数
 * @version 0.1
 * @date 2022-12-30
 *
 * @copyright Copyright (c) 2022
 *
 */
#if !defined(__GLOBALPARAM_HPP)
#define __GLOBALPARAM_HPP
#include "Eigen/Eigen"
#include "MvCameraControl.h"
#include "deque"
#include "opencv2/core.hpp"
#include <cstdint>
#include <opencv2/core/types.hpp>
#pragma pack(1)
typedef struct
{
   
    uint8_t head; // 0x71
    // 电控发送的信息
    float yaw;             // 当前车云台的yaw角，单位为弧度制
    float pitch;           // 当前车云台的pitch角，单位为弧度制
    uint8_t status;        // 状态位，/5==0自己为红色，/5==0自己为蓝色，%5==0为自瞄，%5==1为小符，%5==3为大符
    uint16_t bullet_v;     // 上一次发射的弹速，单位为米每秒
    uint8_t armor_flag;    // 敌方3-5号车是否为大装甲板(即平衡车)，为二进制转十进制，如345全为平衡，为111，输入7，只有3为平衡，为100，输入4
    uint32_t predict_time; // 在电控时间戳为predict_time开火
    float x_a;             // 装甲板在世界坐标系(云台pitch、yaw为0时的相机坐标系)下的x坐标
    float y_a;             // 装甲板在世界坐标系下的y坐标
    float z_a;             // 装甲板在世界坐标系下的z坐标
    float x_c;             // 车中心点在世界坐标系下的x坐标
    float y_c;             // 车中心点在世界坐标系下的y坐标
    float z_c;             // 车中心点在世界坐标系下的z坐标
    float yaw_a;           // 装甲板与车中心点连线与世界坐标系x轴夹角，装甲板正对时为0，车顺时针旋转为负，反之为正，范围大约在-pi/2至pi/2，
    float vx_c;            // 车中心点在世界坐标系下的x速度
    float vy_c;            // 车中心点在世界坐标系下的y速度
    float vz_c;            // 车中心点在世界坐标系下的z速度
    float vyaw_a;          // 如上定义的角，车旋转的角速度
    uint16_t crc;
    uint8_t tail; // 0x4C
    
} MessData;
#pragma pack()
typedef union
{
    MessData message;
    char data[64];
} Translator;
struct Armor
{
    int color;
    int type;
    cv::Point3f center;
    cv::Point3f angle;
    cv::Point2f apex[4];
    double distance_to_image_center;
    Eigen::Vector3d position;
    double yaw;
};
struct Armors
{
    std::deque<Armor> armors;
};
struct ArmorObject
{
    cv::Point2f apex[4];
    cv::Rect_<float> rect;
    int cls;
    int color;
    int area;
    float prob;
    std::vector<cv::Point2f> pts;
};
struct WMObject
{
    cv::Point2f apex[4];
    cv::Rect_<float> rect;
    int cls;
    int color;
    int area;
    float prob;
    std::vector<cv::Point2f> pts;
};
enum COLOR
{
    RED = 0,
    BLUE = 1
};
enum TARGET
{
    SMALL_ARMOR = 0,
    BIG_ARMOR = 1
};

enum ATTACK_MODE
{
    ENERGY = 0,
    ARMOR = 1
};

enum SWITCH
{
    OFF = 0,
    ON = 1
};

enum GETARMORMODE
{
    HIERARCHY = 0,
    FLOODFILL = 1
};

/**
 * @brief 全局参数结构体
 *
 *
 */
struct GlobalParam
{
    //================全局部分================//

    // 当前颜色
    int color = BLUE;
    // 当前获取armor中心点方法
    int get_armor_mode = FLOODFILL;
    // 调试信息，INFO等级的日志是否输出
    int switch_INFO = ON;
    // 调试信息，ERROR等级的日志是否输出
    int switch_ERROR = ON;
    //================相机部分================//
    int attack_mode = ARMOR;
    // 当前使用的相机序号，只连接一个相机时为0，多个相机在取流时可通过cam_index选择设备列表中对应顺序的相机
    int cam_index = 0;
    //===曝光时间===//
    MV_CAM_EXPOSURE_AUTO_MODE enable_auto_exp = MV_EXPOSURE_AUTO_MODE_OFF;
    float energy_exp_time = 200.0F;
    float armor_exp_time = 290.0F;
    float height = 1080;
    float width = 1440;
    //===白平衡===/
    // 默认为目标为红色的白平衡
    // 红色通道
    int r_balance = 1500;
    // 绿色通道
    int g_balance = 1024;
    // 蓝色通道
    int b_balance = 4000;
    // 红色通道
    int e_r_balance = 1500;
    // 绿色通道
    int e_g_balance = 1024;
    // 蓝色通道
    int e_b_balance = 4000;
    //===以下参数重点参与实际帧率的调节===//
    // 图像格式，设置为Bayer RG8，更多图像格式可前往MVS的SDK中寻找
    unsigned int pixel_format = PixelType_Gvsp_BayerRG8;
    // 在经过测试之后，包括在官方SDK中没有调整Acquisition Frame Rate Control Enable的参数，不过在MVS软件中调试时此选项关闭后依然可以达到期望帧率
    //===其他参数===//
    // 自动相机增益使能
    MV_CAM_GAIN_MODE enable_auto_gain = MV_GAIN_MODE_OFF;
    // 相机增益值
    float gain = 17.0F;
    // 相机伽马值，只有在伽马修正开启后有效
    float gamma_value = 0.7F;
    // 触发方式，从0至3依次为触发上升沿、下降沿、高电平、低电平
    int trigger_activation = 0;
    // 设置帧率，仅在不设置外触发时起效
    float frame_rate = 180.0F;
    // 触发模式，ON为外触发，OFF为内触发
    MV_CAM_TRIGGER_MODE enable_trigger = MV_TRIGGER_MODE_OFF;
    // 触发源
    MV_CAM_TRIGGER_SOURCE trigger_source = MV_TRIGGER_SOURCE_LINE0;
    //===============打符识别部分==============//

    //====取图蒙板参数====//
    // 蒙板左上角相对x坐标倍数，范围0～1，TL即Left Top
    float mask_TL_x = 0.125F;
    // 蒙板左上角相对y坐标倍数，范围0～1，TL即Left Top
    float mask_TL_y = 0.0F;
    // 蒙板矩形相对宽度倍数，范围0～1-mask_TL_x
    float mask_width = 0.5F;
    // 蒙板矩形相对高度倍数，范围0～1-mask_TL_y
    float mask_height = 1.0F;

    //====HSV二值化参数====//
    int hmin = 84;  //<! l第一个最小值
    int hmax = 101; //<! l第一个最大值
    int smin = 36;  //<! s最小值
    int smax = 255; //<! s最大值
    int vmin = 46;  //<! v最小值
    int vmax = 255; //<! v最大值
    int e_hmin = 0;
    int e_hmax = 20;
    int e_smin = 35;
    int e_smax = 255;
    int e_vmin = 180;
    int e_vmax = 255;
    //====滤波开关====//
    int switch_gaussian_blur = ON;

    //====UI开关====//
    int switch_UI_contours = ON;
    int switch_UI_areas = ON;
    int switch_UI = ON;

    //====识别参数====//
    int s_armor_min = 900;                    //<! 装甲板最小面积
    int s_armor_max = 6000;                   //<! 装甲板最大面积
    float armor_ratio_min = 1.3F;             //<! 装甲板最小长宽比(长/宽)，值可能为1.6
    float armor_ratio_max = 2.1F;             //<! 装甲板最大长宽比(长/宽)
    float s_armor_ratio_min = 0.5F;           //<! 装甲板最小面积比(轮廓面积/最小外包矩形面积)
    float s_armor_ratio_max = 1.0F;           //<! 装甲板最大面积比(轮廓面积/最小外包矩形面积)
    int s_winghat_min = 600;                  //<! 扇页上半部分最小面积
    int s_winghat_max = 800;                  //<! 扇页上半部分最大面积
    float winghat_ratio_min = 2.0F;           //<! 扇页上半部分最小长宽比(长/宽)
    float winghat_ratio_max = 3.0F;           //<! 扇页上半部分最大长宽比(长/宽)
    float s_winghat_ratio_min = 0.25F;        //<! 扇页上半部分最小面积比(轮廓面积/最小外包矩形面积)
    float s_winghat_ratio_max = 0.4F;         //<! 扇页上半部分最大面积比(轮廓面积/最小外包矩形面积)
    int s_armor_min_floodfill = 900;          //<! 使用漫水处理时装甲板最小面积
    int s_armor_max_floodfill = 6000;         //<! 使用漫水处理时装甲板最大面积
    float armor_ratio_min_floodfill = 0.7F;   //<! 使用漫水处理时装甲板最小长宽比(长/宽)
    float armor_ratio_max_floodfill = 2.0F;   //<! 使用漫水处理时装甲板最大长宽比(长/宽)
    float s_armor_ratio_min_floodfill = 0.5F; //<! 使用漫水处理时装甲板最小面积比(轮廓面积/最小外包矩形面积)
    float s_armor_ratio_max_floodfill = 1.0F; //<! 使用漫水处理时装甲板最大面积比(轮廓面积/最小外包矩形面积)
    int s_R_min = 250;                        //<! R最小面积
    int s_R_max = 1900;                       //<! R最大面积，值可能为750
    float R_ratio_min = 0.99F;                //<! R最小长宽比(长/宽)
    float R_ratio_max = 1.5F;                 //<! R最大长宽比(长/宽)
    float s_R_ratio_min = 0.5F;               //<! R最小面积比(轮廓面积/最小外包矩形面积)
    float s_R_ratio_max = 1.0F;               //<! R最大面积比(轮廓面积/最小外包矩形面积)
    float R_circularity_min = 0.6;            //<! R最小圆度
    float R_circularity_max = 0.75;           //<! R最大圆度
    float R_compactness_min = 15;             //<! R最小紧致度
    float R_compactness_max = 25;             //<! R最大紧致度
    int s_wing_min = 3000;                    //<! 扇页最小面积
    int s_wing_max = 9500;                    //<! 扇页最大面积
    float wing_ratio_min = 1.8F;              //<! 扇页最小长宽比(长/宽)
    float wing_ratio_max = 2.5F;              //<! 扇页最大长宽比(长/宽)
    float s_wing_ratio_min = 0.3F;            //<! 扇页最小面积比(轮廓面积/最小外包矩形面积)
    float s_wing_ratio_max = 0.6F;            //<! 扇页最大面积比(轮廓面积/最小外包矩形面积)
    int R = 700;                              //<! 能量机关半径 mm
    float length = 0.68F;                     //<! 能量机关实际宽度
    float hight = 1.2F;                       //<! 云台标准位置离R高度
    float hit_dx = 6.50F;                     //<! 打击点距能量机关R水平距离
    float constant_speed = 60.0F;             //<! 匀速转动角速度（小符）
    int direction = 1;                        //<! 是否逆时针转动 1是-1不是
    float init_k_ = 0.02F;                    //<! 空气摩擦系数
    //===速度函数参数===//
    float A = 0.912F; //<! A
    float w = 1.942F; //<! w
    float fai = 0.0F; //<! fai
    //===膨胀操作参数===//
    float dialte1 = 5.0F; //<! 第一次膨胀的参数
    float dialte2 = 5.0F; //<! 第二次膨胀的参数
    float dialte3 = 5.0F; //<! 第三次膨胀的参数
    //===预测偏置参数===//
    float re_time = 0.21F; //<! 0.06~0.1约等于半个装甲板
    float thb = 60.0F;     //<! 二值化下阈值
    float tht = 108.0F;    //<! 二值化上阈值
    //===预测部分===//
    float delta_t = 0.3F;

    //===预测部分===//

    int gap = 0;
    int gap_control = 1;
    float min_bullet_v = 16;
    //===相机参数===//(打符)
    // 内参矩阵参数
    double cx = 718.2889; //<! cx
    double cy = 560.3110; //<! cy
    double fx = 2673.8; //<! fx
    double fy = 2673.2; //<! fy
    //畸变矩阵参数
    double k1 = -0.2097;
    double k2 = -0.4538;
    double k3 = 0;
    double p1 = 0;
    double p2 = 0;

    //===R感兴趣区域===//
    float R_roi_xl = 0.28F;  //<! 左边界倍率
    float R_roi_yt = 0.65F;  //<! 上边界倍率
    float R_roi_xr = 0.415F; //<! 右边界倍率
    float R_roi_yb = 0.37F;  //<! 下边界倍率
    cv::Mat K = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    cv::Mat distort_coeffs = (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, k3);
    int list_size = 30; //<! 时间、速度、角速度队列的大小
    //============================自瞄相关参数===============================//
    double threshold_low = 250;
    int armorStat = 0;
    bool isBigArmor[6] = {0, 1, 0, 0, 0, 0};
    //============================信息管理参数================================//
    int message_hold_threshold = 5;
    float fake_pitch = 0.0F;
    float fake_yaw = 1.0F;
    float fake_bullet_v = 25.0F;
    uint8_t fake_status = 0;
    float fake_now_time = 0;
    float fake_predict_time = 0;
};
#endif // __GLOBALPARAM_HPP