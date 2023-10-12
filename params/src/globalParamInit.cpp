#include "globalParamInit.hpp"
#include <iostream>
void initGlobalParam(GlobalParam &gp, address &addr, int color)
{
    cv::FileStorage fs;
    if (color == RED)
    {
        fs.open(addr.yaml_address + "WMConfigRed.yaml", cv::FileStorage::READ);
    }
    else if (color == BLUE)
    {
        fs.open(addr.yaml_address + "WMConfigBlue.yaml", cv::FileStorage::READ);
    }
    fs["color"] >> gp.color;
    fs["switch_INFO"] >> gp.switch_INFO;
    fs["switch_ERROR"] >> gp.switch_ERROR;
    fs["get_armor_mode"] >> gp.get_armor_mode;
    fs["cam_index"] >> gp.cam_index;
    fs["enable_auto_exp"] >> gp.enable_auto_exp;
    fs["energy_exp_time"] >> gp.energy_exp_time;
    fs["armor_exp_time"] >> gp.armor_exp_time;
    fs["r_balance"] >> gp.r_balance;
    fs["g_balance"] >> gp.g_balance;
    fs["b_balance"] >> gp.b_balance;
    fs["e_r_balance"] >> gp.e_r_balance;
    fs["e_g_balance"] >> gp.e_g_balance;
    fs["e_b_balance"] >> gp.e_b_balance;
    fs["enable_auto_gain"] >> gp.enable_auto_gain;
    fs["gain"] >> gp.gain;
    fs["gamma_value"] >> gp.gamma_value;
    fs["trigger_activation"] >> gp.trigger_activation;
    fs["frame_rate"] >> gp.frame_rate;
    fs["enable_trigger"] >> gp.enable_trigger;
    fs["trigger_source"] >> gp.trigger_source;
    fs["mask_TL_x"] >> gp.mask_TL_x;
    fs["mask_TL_y"] >> gp.mask_TL_y;
    fs["mask_width"] >> gp.mask_width;
    fs["mask_height"] >> gp.mask_height;
    fs["hmin"] >> gp.hmin;
    fs["hmax"] >> gp.hmax;
    fs["smin"] >> gp.smin;
    fs["smax"] >> gp.smax;
    fs["vmin"] >> gp.vmin;
    fs["vmax"] >> gp.vmax;
    fs["e_hmin"] >> gp.e_hmin;
    fs["e_hmax"] >> gp.e_hmax;
    fs["e_smin"] >> gp.e_smin;
    fs["e_smax"] >> gp.e_smax;
    fs["e_vmin"] >> gp.e_vmin;
    fs["e_vmax"] >> gp.e_vmax;
    fs["switch_gaussian_blur"] >> gp.switch_gaussian_blur;
    fs["s_armor_min"] >> gp.s_armor_min;
    fs["s_armor_max"] >> gp.s_armor_max;
    fs["armor_ratio_min"] >> gp.armor_ratio_min;
    fs["armor_ratio_max"] >> gp.armor_ratio_max;
    fs["s_armor_ratio_min"] >> gp.s_armor_ratio_min;
    fs["s_armor_ratio_max"] >> gp.s_armor_ratio_max;
    fs["s_winghat_min"] >> gp.s_winghat_min;
    fs["s_winghat_max"] >> gp.s_winghat_max;
    fs["s_winghat_ratio_min"] >> gp.s_winghat_ratio_min;
    fs["s_winghat_ratio_max"] >> gp.s_winghat_ratio_max;
    fs["winghat_ratio_min"] >> gp.winghat_ratio_min;
    fs["winghat_ratio_max"] >> gp.winghat_ratio_max;
    fs["s_armor_min_floodfill"] >> gp.s_armor_min_floodfill;
    fs["s_armor_max_floodfill"] >> gp.s_armor_max_floodfill;
    fs["R_circularity_min"] >> gp.R_circularity_min;
    fs["R_circularity_max"] >> gp.R_circularity_max;
    fs["R_compactness_min"] >> gp.R_compactness_min;
    fs["R_compactness_max"] >> gp.R_compactness_max;
    fs["armor_ratio_min_floodfill"] >> gp.armor_ratio_min_floodfill;
    fs["armor_ratio_max_floodfill"] >> gp.armor_ratio_max_floodfill;
    fs["s_armor_ratio_min_floodfill"] >> gp.s_armor_ratio_min_floodfill;
    fs["s_armor_ratio_max_floodfill"] >> gp.s_armor_ratio_max_floodfill;
    fs["s_R_min"] >> gp.s_R_min;
    fs["s_R_max"] >> gp.s_R_max;
    fs["R_ratio_min"] >> gp.R_ratio_min;
    fs["R_ratio_max"] >> gp.R_ratio_max;
    fs["s_R_ratio_min"] >> gp.s_R_ratio_min;
    fs["s_R_ratio_max"] >> gp.s_R_ratio_max;
    fs["s_wing_min"] >> gp.s_wing_min;
    fs["s_wing_max"] >> gp.s_wing_max;
    fs["wing_ratio_min"] >> gp.wing_ratio_min;
    fs["wing_ratio_max"] >> gp.wing_ratio_max;
    fs["s_wing_ratio_min"] >> gp.s_wing_ratio_min;
    fs["s_wing_ratio_max"] >> gp.s_wing_ratio_max;
    fs["R"] >> gp.R;
    fs["length"] >> gp.length;
    fs["hight"] >> gp.hight;
    fs["hit_dx"] >> gp.hit_dx;
    fs["constant_speed"] >> gp.constant_speed;
    fs["direction"] >> gp.direction;
    fs["init_k_"] >> gp.init_k_;
    fs["A"] >> gp.A;
    fs["w"] >> gp.w;
    fs["fai"] >> gp.fai;
    fs["dialte1"] >> gp.dialte1;
    fs["dialte2"] >> gp.dialte2;
    fs["dialte3"] >> gp.dialte3;
    fs["re_time"] >> gp.re_time;
    fs["thb"] >> gp.thb;
    fs["tht"] >> gp.tht;

    fs["R_roi_xl"] >> gp.R_roi_xl;
    fs["R_roi_xr"] >> gp.R_roi_xr;
    fs["R_roi_yb"] >> gp.R_roi_yb;
    fs["R_roi_yt"] >> gp.R_roi_yt;
    fs["list_size"] >> gp.list_size;
    fs["switch_UI_contours"] >> gp.switch_UI_contours;
    fs["switch_UI_areas"] >> gp.switch_UI_areas;
    fs["switch_UI"] >> gp.switch_UI;
    fs["fake_pitch"] >> gp.fake_pitch;
    fs["fake_yaw"] >> gp.fake_yaw;
    fs["fake_bullet_v"] >> gp.fake_bullet_v;
    fs["fake_status"] >> gp.fake_status;
    fs["fake_now_time"] >> gp.fake_now_time;
    fs["fake_predict_time"] >> gp.fake_predict_time;
    LOG_IF(INFO, gp.switch_INFO) << "initGlobalParam Successful";
}

void saveGlobalParam(GlobalParam &gp, address &add, int &color)
{
    cv::FileStorage fs;
    if (color == RED)
    {
        fs.open(add.yaml_address + "WMConfigRed.yaml", cv::FileStorage::WRITE);
    }
    else if (color == BLUE)
    {
        fs.open(add.yaml_address + "WMConfigBlue.yaml", cv::FileStorage::WRITE);
    }
    fs << "color" << gp.color;
    fs << "switch_INFO" << gp.switch_INFO;
    fs << "switch_ERROR" << gp.switch_ERROR;
    fs << "get_armor_mode" << gp.get_armor_mode;
    fs << "cam_index" << gp.cam_index;
    fs << "enable_auto_exp" << gp.enable_auto_exp;
    fs << "energy_exp_time" << gp.energy_exp_time;
    fs << "armor_exp_time" << gp.armor_exp_time;
    fs << "r_balance" << gp.r_balance;
    fs << "g_balance" << gp.g_balance;
    fs << "b_balance" << gp.b_balance;
    fs << "enable_auto_gain" << gp.enable_auto_gain;
    fs << "gain" << gp.gain;
    fs << "gamma_value" << gp.gamma_value;
    fs << "trigger_activation" << gp.trigger_activation;
    fs << "frame_rate" << gp.frame_rate;
    fs << "enable_trigger" << gp.enable_trigger;
    fs << "trigger_source" << gp.trigger_source;
    fs << "mask_TL_x" << gp.mask_TL_x;
    fs << "mask_TL_y" << gp.mask_TL_y;
    fs << "mask_width" << gp.mask_width;
    fs << "mask_height" << gp.mask_height;
    fs << "hmin" << gp.hmin;
    fs << "hmax" << gp.hmax;
    fs << "smin" << gp.smin;
    fs << "smax" << gp.smax;
    fs << "vmin" << gp.vmin;
    fs << "vmax" << gp.vmax;
    fs << "switch_gaussian_blur" << gp.switch_gaussian_blur;
    fs << "s_armor_min" << gp.s_armor_min;
    fs << "s_armor_max" << gp.s_armor_max;
    fs << "armor_ratio_min" << gp.armor_ratio_min;
    fs << "armor_ratio_max" << gp.armor_ratio_max;
    fs << "s_armor_ratio_min" << gp.s_armor_ratio_min;
    fs << "s_armor_ratio_max" << gp.s_armor_ratio_max;
    fs << "s_winghat_min" << gp.s_winghat_min;
    fs << "s_winghat_max" << gp.s_winghat_max;
    fs << "s_winghat_ratio_min" << gp.s_winghat_ratio_min;
    fs << "s_winghat_ratio_max" << gp.s_winghat_ratio_max;
    fs << "winghat_ratio_min" << gp.winghat_ratio_min;
    fs << "winghat_ratio_max" << gp.winghat_ratio_max;
    fs << "s_armor_min_floodfill" << gp.s_armor_min_floodfill;
    fs << "s_armor_max_floodfill" << gp.s_armor_max_floodfill;
    fs << "R_circularity_min" << gp.R_circularity_min;
    fs << "R_circularity_max" << gp.R_circularity_max;
    fs << "R_compactness_min" << gp.R_compactness_min;
    fs << "R_compactness_max" << gp.R_compactness_max;
    fs << "armor_ratio_min_floodfill" << gp.armor_ratio_min_floodfill;
    fs << "armor_ratio_max_floodfill" << gp.armor_ratio_max_floodfill;
    fs << "s_armor_ratio_min_floodfill" << gp.s_armor_ratio_min_floodfill;
    fs << "s_armor_ratio_max_floodfill" << gp.s_armor_ratio_max_floodfill;
    fs << "s_R_min" << gp.s_R_min;
    fs << "s_R_max" << gp.s_R_max;
    fs << "R_ratio_min" << gp.R_ratio_min;
    fs << "R_ratio_max" << gp.R_ratio_max;
    fs << "s_R_ratio_min" << gp.s_R_ratio_min;
    fs << "s_R_ratio_max" << gp.s_R_ratio_max;
    fs << "s_wing_min" << gp.s_wing_min;
    fs << "s_wing_max" << gp.s_wing_max;
    fs << "wing_ratio_min" << gp.wing_ratio_min;
    fs << "wing_ratio_max" << gp.wing_ratio_max;
    fs << "s_wing_ratio_min" << gp.s_wing_ratio_min;
    fs << "s_wing_ratio_max" << gp.s_wing_ratio_max;
    fs << "R" << gp.R;
    fs << "length" << gp.length;
    fs << "hight" << gp.hight;
    fs << "hit_dx" << gp.hit_dx;
    fs << "constant_speed" << gp.constant_speed;
    fs << "direction" << gp.direction;
    fs << "init_k_" << gp.init_k_;
    fs << "A" << gp.A;
    fs << "w" << gp.w;
    fs << "fai" << gp.fai;
    fs << "dialte1" << gp.dialte1;
    fs << "dialte2" << gp.dialte2;
    fs << "dialte3" << gp.dialte3;
    fs << "re_time" << gp.re_time;
    fs << "thb" << gp.thb;
    fs << "tht" << gp.tht;
    fs << "cx" << gp.cx;
    fs << "cy" << gp.cy;
    fs << "fx" << gp.fx;
    fs << "fy" << gp.fy;
    fs << "R_roi_xl" << gp.R_roi_xl;
    fs << "R_roi_xr" << gp.R_roi_xr;
    fs << "R_roi_yb" << gp.R_roi_yb;
    fs << "R_roi_yt" << gp.R_roi_yt;
    fs << "list_size" << gp.list_size;
    LOG_IF(INFO, gp.switch_INFO) << "saveGlobalParam Successful";
}