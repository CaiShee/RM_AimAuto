//========================================//
#include <CameraParams.h>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <glog/logging.h>
#include <iomanip>
#include <mutex>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <pthread.h>
#include <sstream>
#include <string>
#include <unistd.h>
//========================================//
#include "AimAuto.hpp"
#include "MessageManager.hpp"
#include "SerialPort.hpp"
#include "UIManager.hpp"
#include "WMIdentify.hpp"
#include "WMPredict.hpp"
#include "camera.hpp"
#include "globalParam.hpp"
#include "globalParamInit.hpp"
#include "globalText.hpp"
#include "monitor.hpp"
//========================================//
#define PI 3.14159

// 全局变量
// 全局变量参数，这个参数存储着全部的需要的参数
GlobalParam gp;
MessageManager MManager(gp);
// 全局地址参数，这个参数存储着全部的需要的地址
address addr;
// 全局图片，负责将图片从取流的读取线程读取到运算线程
cv::Mat pic;
// 读信息，是从电控接受的信息，在读线程被赋值，之后交给运算线程
Translator translator;
// 临时信息，为了防止数据的堵塞，使用临时信息反复的读取，在需要真正获取信息的时候将信息传给translator
Translator temp;
// 声明线程
// 读线程，负责读取串口信息以及取流
void *ReadFunction(void *arg);
// 运算线程，负责对图片进行处理，并且完成后续的要求
void *OperationFunction(void *arg);
int main(int argc, char **argv)
{
    // 初始化Glog并设置部分标志位
    google::InitGoogleLogging(argv[0]);
    // 设置Glog输出的log文件写在address中log_address对应的地址下
    FLAGS_log_dir = addr.log_address;
    printf("welcome\n");
    // 实例化通信串口类
    SerialPort *serialPort = new SerialPort(argv[1]);
    // 设置通信串口对象初始值
    serialPort->InitSerialPort(int(*argv[2] - '0'), 8, 1, 'N');
#ifndef NOPORT
    MManager.read(temp, *serialPort);
#ifdef THREADANALYSIS
    printf("init status is: %d\n", temp.message.status);
#endif
    // 通过电控发来的标志位是0～4还是5～9来确定是红方还是蓝方，其中0～4是红方，5～9是蓝方
    MManager.initParam(temp.message.status / 5 == 0 ? RED : BLUE);
#else
    // 再没有串口的时候直接设定颜色，这句代码可以根据需要进行更改
    MManager.initParam(RED);
    translator.message.predict_time = 0;
#endif // NOPORT
    // 创建三个线程
    pthread_t readThread;
    pthread_t WMIThread;
    // 开启线程
    pthread_create(&readThread, NULL, ReadFunction, serialPort);
    pthread_create(&WMIThread, NULL, OperationFunction, serialPort);
    pthread_join(WMIThread, NULL);
    return 0;
}

void *ReadFunction(void *arg)
{
#ifdef THREADANALYSIS
    printf("read function init successful\n");
#endif
    // 传入的参数赋给串口，以获得串口数据
    SerialPort *serialPort = (SerialPort *)arg;
    int len = 0;
    while (1)
    {
        MManager.read(temp, *serialPort);
        usleep(3000);
    }
    return NULL;
}

void *OperationFunction(void *arg)
{
#ifdef THREADANALYSIS
    printf("operation function init successful\n");
#endif

    //=================初始化对象==================//

    SerialPort *serialPort = (SerialPort *)arg;
    // 实例化能量机关识别类
    WMIdentify WMI(gp);
    // 实例化自瞄类
    AimAuto aim(&gp);
    // 实例化UI类
    UIManager UI;
    // 重置能量机关识别类
    WMI.clear();
    // 实例化能量机关预测类
    WMIPredict WMIPRE;

#ifndef VIRTUALGRAB
    // 假如是现实取流，初始化相机
    Camera camera(gp);
    camera.init();
#endif // VIRTUALGRAB

    //===========================================//

    //======动态调参使用参数======//

#ifdef DEBUGMODE
    // 当前按键
    int key = 0;
    // debug时waitKey时间，也就是整体的运行速率
    int debug_t = 100;
    // 储存相机坐标系下的点，用于绘图
    std::deque<cv::Point3f> points3d;
    // 储存当前时间，用于绘图
    std::deque<double> times;
#endif // DEBUGMODE

    //========================//

    while (1)
    {
#ifdef THREADANALYSIS
        std::cout << "==========================" << std::endl;
        std::chrono::milliseconds start_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch());
#endif
        int ret = 1;
#ifndef NOPORT

        MManager.copy(temp, translator);
        ret = MManager.CheckCrc(translator, 61);
        ret = 1;

#else

        // 假如没有串口，手动设置串口信息，其中时间每次自加0.02，这是通信的间隔
        uint16_t currectCrc = 0;
        uint16_t nowCrc = 0;
        int len = 64;
        MManager.FakeMessage(translator);

#endif // NOPORT \
       // translator.message.status = 1;
        if (ret)
        {

#ifndef NOPORT
            if (translator.message.status / 5 != gp.color)
                initGlobalParam(gp, addr, translator.message.status / 5);
            if (translator.message.armor_flag != gp.armorStat)
                MManager.ChangeBigArmor(translator);
#endif
            translator.message.status = 5;
            if (translator.message.status % 5 != 0)
            {
#ifndef VIRTUALGRAB
                camera.change_attack_mode(ENERGY, gp);
#endif
                gp.attack_mode = ENERGY;
            }
            else
            {
#ifndef VIRTUALGRAB
                camera.change_attack_mode(ARMOR, gp);
#endif
                gp.attack_mode = ARMOR;
            }

#ifndef VIRTUALGRAB
            camera.getFrame(pic);
#else
            MManager.getFrame(pic, translator);
#endif
        }
        else
        {
#ifdef THREADANALYSIS
            printf("crc was wrong\n");
#endif
            continue;
        }

        // 如果图片为空，不执行
        if (pic.empty() == 1)
        {
#ifdef THREADANALYSIS
            printf("pic is empty\n");
#endif
            // 图片使用完毕，图片已读未写
            // exit(-1);
            continue;
        }
#ifdef RECORDVIDEO
        MManager.recordFrame(pic);
#endif
        // 打击小符或者大符
        static int fit_control = 1;
        if (translator.message.status % 5 == 0)
        {
            fit_control = 1;
            WMI.clear();
            std::cout << WMI.getAngleVelocityList().size() << std::endl;
        }

        if (translator.message.status % 5 == 1 || translator.message.status % 5 == 3)
        {
#ifdef DEBUGMODE
            times.push_back((double)translator.message.predict_time / 1000);
#endif // DEBUGMODE
            translator.message.bullet_v /= 10;
            if (translator.message.bullet_v < gp.min_bullet_v)
            {
                translator.message.status = 102;
                std::cout << "弹速小，为：" << translator.message.bullet_v << std::endl;
                continue;
            }
#ifdef THREADANALYSIS
            printf("status == 1 || status == 3\n");
#endif

#ifdef USEWMNET
            // 进行识别

            WMI.startWMINet(pic, translator);

#else

            WMI.startWMIdentify(pic, translator);

#endif

#ifdef DEBUGMODE
            // 如果开启DEBUGMODE，使用UI类在图片上绘制UI
            UI.receive_pic(pic);
            // 通过按键进行调参，这里的顺序必须是先这个再按键
            UI.windowsManager(gp, key, debug_t);
#ifdef THREADANALYSIS
            printf("picture showed\n");
#endif
            cv::imshow("result", pic);

            // 获取按键，用于动态调参
            key = cv::waitKey(debug_t);
            if (key == ' ')
                cv::waitKey(0);
            if (key == 27)
                return nullptr;
#endif // DEBUGMODE

            // 如果ListStat为0，则UpdateList失败，没有新数据，本次识别失败，不进入预测
            if (WMI.getListStat() == 0)
            {
#ifdef THREADANALYSIS
                printf("WMI failed\n");
#endif
                translator.message.status = 102;
#ifdef DEBUGHIT
                WMIPRE.GiveDebugImg(WMI.getImg0());
#endif
            }
            //=========================================//
            // 预测部分
            else
            {
                // std::cout<<WMI.getDirection()<<std::endl;
                WMIPRE.UpdateData(WMI.getDirection(), WMI.getRadius(), WMI.getR_center(), WMI.getImg0(), WMI.getData_img(), translator);
            }

            // 如果击打大符

            if (translator.message.status % 5 == 3)
            {

                // std::cout<<WMI.getAngleVelocityList().size()<<std::endl;
                //  如果角速度数据数量够，进行拟合

                if (WMI.getAngleVelocityList().size() >= gp.list_size)
                {

                    if (gp.gap % gp.gap_control == 0 && fit_control == 1)
                    {
                        static int countv = 0;

                        WMIPRE.Fit(WMI.getTimeList(), WMI.getAngleVelocityList(), gp, translator);

                        fit_control = 1;
                    }

                } //
                // 否则进入下一次循环，积累数据
                else
                    continue;
                if (translator.message.status % 5 == 3)
                {
                    WMIPRE.NewtonDspBig(WMI.getAngleList(), translator, gp, WMI.getYaw());
                    WMIPRE.ResultLog(translator, gp, WMI.getYaw());
                } // 大符预测，计算pitch和yaw
            }
            // 如果击打小符
            if (translator.message.status % 5 == 1)
            {
                WMIPRE.NewtonDspSmall(WMI.getAngleList(), translator, gp, WMI.getYaw());
                WMIPRE.ResultLog(translator, gp, WMI.getYaw());
            }
            //  输出更加完整的预测日志

#ifdef DEBUGHIT
            if (1)
            {

                std::chrono::milliseconds end_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch());
                float total_time = (end_ms - start_ms).count() / 1000.0;
                cv::Mat debug_img = WMIPRE.GetDebugImg();
                cv::putText(debug_img, "fps:" + std::to_string(1 / total_time), cv::Point(30, 210), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 100, 0));
                cv::imshow("Energy_debug", debug_img);
                cv::waitKey(1);
                // std::cout << "fps: " << 1 / total_time << std::endl;
            }
#endif
        }

        else if (translator.message.status % 5 == 0 || translator.message.status % 5 == 2)
        {
#ifdef THREADANALYSIS
            printf("status == 0\n");
#endif
#ifdef DEBUGMODE
            times.push_back((double)translator.message.predict_time / 1000);
#endif // DEBUGMODE

            aim.AimAutoYHY(pic, translator);
            aim.NewTracker(translator, pic);
            MManager.HoldMessage(translator);

#ifdef DEBUGMODE
            drawStat(points3d, times, translator);
            UI.receive_pic(pic);
            UI.windowsManager(gp, key, debug_t);
            cv::imshow("result", pic);
#endif // DEBUGMODE
#ifdef THREADANALYSIS
            printf("picture showed\n");
            std::chrono::milliseconds end_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch());
            float total_time = (end_ms - start_ms).count() / 1000.0;
            std::cout << "fps:" << 1 / total_time << std::endl;
#endif
#ifdef DEBUGMODE
            // resizePic(pic);
            //================7_28=================
            // 获取按键，用于动态调参
            key = cv::waitKey(debug_t);
            if (key == ' ')
                key = cv::waitKey(0);
            if (key == 27)
                return nullptr;
#endif // DEBUGMODE
        }
        if (translator.message.status == 99)
            return nullptr;
#ifndef NOPORT
        MManager.UpdateCrc(translator, 61);
        MManager.write(translator, *serialPort);
#endif // NOPORT
    }
    return NULL;
}