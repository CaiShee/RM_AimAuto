#if !defined(__GLOBALPARAMINIT_HPP)
#define __GLOBALPARAMINIT_HPP
#include "globalParam.hpp"
#include "globalText.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <glog/logging.h>
#include <opencv2/video/video.hpp>
#include <string.h>

/**
 * @brief 初始化全局参数结构结构体
 *
 * @param gp 等待初始化的全局参数结构体
 * @param add 通过address结构体传入配置文件的地址
 * @param color 当前颜色
 */
void initGlobalParam(GlobalParam &gp, address &addr, int color);

void saveGlobalParam(GlobalParam &, address &, int &);

#endif // __GLOBALPARAMINIT_HPP