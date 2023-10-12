#include "CameraParams.h"
#include "Eigen/Eigen"
#include "Eigen/src/Core/Matrix.h"
#include "gaoning.hpp"
#include "globalParam.hpp"
#include "globalText.hpp"
#include "monitor.hpp"
#include "opencv2/calib3d.hpp"
#include <AimAuto.hpp>
#include <cmath>
#include <cstdio>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ostream>
#include <string>

#define VECTOR_X 0.11
#define VECTOR_Y 0
#define VECTOR_Z 0.095

AimAuto::AimAuto(GlobalParam *gp) : dt(0.0)
{
    det = initDetector(gp->color);
    this->restart_time = 0;
    // EKF
    // xa = x_armor, xc = x_robot_center
    // state: xc, yc, zc, yaw, v_xc, v_yc, v_zc, v_yaw, r
    // measurement: xa, ya, za, yaw
    // f - Process function
    this->cnt = 0;
    auto f = [this](const Eigen::VectorXd &x)
    {
        Eigen::VectorXd x_new = x;
        x_new(0) += x(4) * dt;
        x_new(1) += x(5) * dt;
        x_new(2) += x(6) * dt;
        x_new(3) += x(7) * dt;
        return x_new;
    };
    // J_f - Jacobian of process function
    auto j_f = [this](const Eigen::VectorXd &)
    {
        Eigen::MatrixXd f(8, 8);
        // clang-format off
    f <<  1,   0,   0,   0,   dt, 0,   0,    0,
          0,   1,   0,   0,   0,   dt, 0,    0,
          0,   0,   1,   0,   0,   0,   dt,  0,
          0,   0,   0,   1,   0,   0,   0,   dt,
          0,   0,   0,   0,   1,   0,   0,   0,
          0,   0,   0,   0,   0,   1,   0,   0,
          0,   0,   0,   0,   0,   0,   1,   0,
          0,   0,   0,   0,   0,   0,   0,   1;
        // clang-format on
        return f;
    };
    // h - Observation function
    auto h = [this](const Eigen::VectorXd &x)
    {
        Eigen::VectorXd z(4);
        double xc = x(0), yc = x(1), yaw = x(3);
        z(0) = xc - r * cos(yaw); // xa
        z(1) = yc + r * sin(yaw); // ya
        z(2) = x(2);              // za
        z(3) = x(3);              // yaw
        return z;
    };
    // J_h - Jacobian of observation function
    auto j_h = [this](const Eigen::VectorXd &x)
    {
        Eigen::MatrixXd h(4, 8);
        double yaw = x(3);
        // clang-format off
    //    xc   yc   zc   yaw         vxc  vyc  vzc  vyaw r
    h <<  1,   0,   0,   r*sin(yaw), 0,   0,   0,   0,
          0,   1,   0,   r*cos(yaw),0,   0,   0,   0,
          0,   0,   1,   0,          0,   0,   0,   0,
          0,   0,   0,   1,          0,   0,   0,   0;
        // clang-format on
        return h;
    };
    double max_match_distance = 0.7;
    int tracking_threshold = 5;
    int lost_threshold = 5;

    // Q - process noise covariance matrix
    auto q_v = std::vector<double>{
        // xc  yc    zc    yaw   vxc   vyc   vzc   vyaw
        // 1e-4, 1e-4, 1e-1, 25e-4, 4e-2, 4e-2, 1e-2, 4};
        1, 1e-4, 1e-4, 1e-1, 25e-4, 4e-2, 4e-2, 1e-2, 4e-2};

    // R - measurement noise covariance matrix
    auto r_v = std::vector<double>{
        // xa  ya    za    yaw
        // 5e-2, 5e-2, 1e-3, 2e-2
        1e-1, 1.5e-2, 1e-3, 100};

    Eigen::DiagonalMatrix<double, 8> q;
    q.diagonal() << q_v[0], q_v[1], q_v[2], q_v[3], q_v[4], q_v[5], q_v[6], q_v[7];
    Eigen::DiagonalMatrix<double, 4> r;
    r.diagonal() << r_v[0], r_v[1], r_v[2], r_v[3];
    tracker = new Tracker(max_match_distance, tracking_threshold, lost_threshold, q, r);
    // P - error estimate covariance matrix
    Eigen::DiagonalMatrix<double, 8> p0;
    p0.setIdentity();
    tracker->ekf = ExtendedKalmanFilter{f, h, j_f, j_h, q, r, p0};
    this->gp = gp;
    this->tar_list.clear();
}

AimAuto::~AimAuto()
{
}

void AimAuto::AimAutoYHY(cv::Mat &src, Translator &ts)
{
    if (det->detect_color != gp->color)
    {
        det = initDetector(gp->color);
    }
    this->time = (double)ts.message.predict_time;
    this->tar_list.clear();
    auto armors = det->detect(src);
    std::sort(armors.begin(), armors.end(), [&](const rm_auto_aim::Armor &la, const rm_auto_aim::Armor &lb)
              { return abs((double)src.cols / 2 - ((la.left_light.top + la.right_light.top + la.left_light.bottom + la.right_light.bottom) / 4).x) < abs((double)src.cols / 2 - ((lb.left_light.top + lb.right_light.top + lb.left_light.bottom + lb.right_light.bottom) / 4).x); });
    for (auto armor : armors)
    {

        std::cout << armor.number << std::endl;
        int number = -1;
        convertNumber(armor.number, number);
        Armor tar;
        pnp_solve(armor, ts, src, tar, number);
        tar_list.emplace_back(tar);
#ifdef DEBUGMODE
        cv::line(src, tar.apex[0], tar.apex[1], cv::Scalar(193, 182, 255), 3);
        cv::line(src, tar.apex[1], tar.apex[2], cv::Scalar(193, 182, 255), 3);
        cv::line(src, tar.apex[2], tar.apex[3], cv::Scalar(193, 182, 255), 3);
        cv::line(src, tar.apex[3], tar.apex[0], cv::Scalar(193, 182, 255), 3);
        cv::circle(src, (tar.apex[0] + tar.apex[1] + tar.apex[2] + tar.apex[3]) / 4, 5, cv::Scalar(193, 182, 255), -1);
#endif // DEBUGMODE
    }
    armors_msg.armors = tar_list;
    //=====================输出装甲板坐标并且在DEBUGMODE中展示识别到的装甲板======================//
    if (tar_list.size() != 0)
    {
        ts.message.x_a = tar_list[0].position(0) * 1000;
        ts.message.y_a = tar_list[0].position(1) * 1000;
        ts.message.z_a = tar_list[0].position(2) * 1000;
    }
    else
    {
        ts.message.x_a = 0;
        ts.message.y_a = 0;
        ts.message.z_a = 0;
    }
#ifdef DEBUGMODE
    showDist(tar_list, src);
    cv::imshow("result", src);
#endif // DEBUGMODE
}
void AimAuto::NewTracker(Translator &ts, cv::Mat &src)
{
    if (restart_time == 0)
        restart_time = this->time;
    ts.message.armor_flag = 0;
    if (armors_msg.armors.size() == 0)
    {
        ts.message.status = 0;
        return;
    }
    if (!this->updateTracker(ts, src))
        return;
    /*
    // 初始化五秒钟，可以取消
    if (this->time - restart_time < 5000)
    {
        ts.message.x_a = 0;
        ts.message.y_a = 0;
        ts.message.z_a = 0;
        ts.message.x_c = 0;
        ts.message.y_c = 0;
        ts.message.z_c = 0;
        return;
    }
    */

    double real_time = 0, flight_time = 0, react_time = 0.7;
    cv::Point3f target;

    //============魔法参数============//
    double pz, pyaw;
    if ((int)ts.message.status / 5 == 1)
    {
        pz = 250.0;
        pyaw = 0.5;
    }
    else
    {
        pz = 0.0;
        pyaw = -0.3;
    }

    target = findTarget(ts, pz, pyaw, real_time);

    storeMessage(target, ts);

    //============== Hit Outpost Second ================
    std::cout << "static is " << ts.message.status << std::endl;
    if (ts.message.status % 5 != 2)
    {
        drawRobotState(ts, 0.1, real_time, this->r);
    }
    else
    {
        drawStationState(ts, 0.1, real_time, this->r);
    }
#ifdef SENDCAMERA

    this->convertPoint(ts, target);

    this->fireControl(ts, src);

#endif
}

void AimAuto::pnp_solve(rm_auto_aim::Armor &armor, Translator &ts, cv::Mat &src, Armor &tar, int number)
{
    //===============pnp解算===============//

    std::vector<cv::Point3f> objPoints;
    if (!gp->isBigArmor[number]) // 如果是小装甲板
        objPoints = std::vector<cv::Point3f>{
            cv::Point3f(-67.50F, 28.50F, 0), // 2,3,4,1象限顺序
            cv::Point3f(-67.50F, -28.50F, 0),
            cv::Point3f(67.50F, -28.50F, 0),
            cv::Point3f(67.50F, 28.50F, 0),
        };
    else // 如果不是小装甲板
        objPoints = std::vector<cv::Point3f>{
            cv::Point3f(-115.00F, 28.50F, 0), // 2,3,4,1象限顺序
            cv::Point3f(-115.00F, -28.50F, 0),
            cv::Point3f(115.00F, -28.50F, 0),
            cv::Point3f(115.00F, 28.50F, 0),
        };
    tVec.create(3, 1, CV_64F);
    rVec.create(3, 1, CV_64F);
    _K = (cv::Mat_<double>(3, 3) << (float)gp->fx, 0, (float)gp->cx, 0, (float)gp->fy, (float)gp->cy, 0, 0, 1);
    _dist = {(float)gp->k1, (float)gp->k2, (float)gp->p1, (float)gp->p2, (float)gp->k3};
    std::vector<cv::Point2f> tmp = {armor.left_light.top, armor.left_light.bottom, armor.right_light.bottom, armor.right_light.top};
    cv::solvePnP(cv::InputArray(objPoints), cv::InputArray(tmp),
                 cv::InputArray(_K), cv::InputArray(_dist),
                 cv::OutputArray(rVec), cv::OutputArray(tVec), false, cv::SOLVEPNP_ITERATIVE);

    //=========================================//

    //=================坐标系转化================//
    tar.center = cv::Point3f(tVec.at<double>(0), tVec.at<double>(1), tVec.at<double>(2));
    cv::Mat rotation_matrix;
    cv::Rodrigues(rVec, rotation_matrix);
    double yaw = std::atan2(rotation_matrix.at<double>(0, 2), rotation_matrix.at<double>(2, 2));
    if (yaw >= 0)
    {
        yaw = -(3.14 - yaw);
    }
    else
    {
        yaw = 3.14 + yaw;
    }

    //================数据转存===================//

    tar.yaw = yaw;
    tar.angle = cv::Point3f(rVec.at<double>(0), rVec.at<double>(1), rVec.at<double>(2));
    tar.color = det->detect_color;
    tar.type = number;
    tar.apex[0] = armor.left_light.top;
    tar.apex[1] = armor.left_light.bottom;
    tar.apex[2] = armor.right_light.bottom;
    tar.apex[3] = armor.right_light.top;
    tar.distance_to_image_center = abs((double)src.cols / 2 - ((tar.apex[0] + tar.apex[1] + tar.apex[2] + tar.apex[3]) / 4).x);

    //=========================================//

    Eigen::MatrixXd m_pitch(3, 3);
    Eigen::MatrixXd m_yaw(3, 3);
    m_yaw << cos(ts.message.yaw), -sin(ts.message.yaw), 0, sin(ts.message.yaw), cos(ts.message.yaw), 0, 0, 0, 1;
    m_pitch << cos(ts.message.pitch), 0, sin(ts.message.pitch), 0, 1, 0, -sin(ts.message.pitch), 0, cos(ts.message.pitch);
    Eigen::Vector3d temp;
    temp = Eigen::Vector3d(tar.center.z / 1000 + VECTOR_X, -tar.center.x / 1000 + VECTOR_Y, -tar.center.y / 1000 + VECTOR_Z);
    tar.yaw -= ts.message.yaw;

    // gn.Faker(temp, tar.yaw, 6, 0.1, points3);
    tar.position = m_yaw * m_pitch * temp;

    //=========================================//
}

std::unique_ptr<rm_auto_aim::Detector> initDetector(int color)
{
    address addr;
    int binary_thres = 160;
    int detect_color = color;
    double min_ratio,
        max_ratio,
        max_angle_l,
        min_light_ratio,
        min_small_center_distance,
        max_small_center_distance,
        min_large_center_distance,
        max_large_center_distance,
        max_angle_a,
        threshold;
    cv::FileStorage fs;
    fs.open(addr.yaml_address + "detect.yaml", cv::FileStorage::READ);
    fs["min_ratio"] >> min_ratio;
    fs["max_ratio"] >> max_ratio;
    fs["max_angle_l"] >> max_angle_l;
    fs["min_light_ratio"] >> min_light_ratio;
    fs["min_small_center_distance"] >> min_small_center_distance;
    fs["max_small_center_distance"] >> max_small_center_distance;
    fs["min_large_center_distance"] >> min_large_center_distance;
    fs["max_large_center_distance"] >> max_large_center_distance;
    fs["max_angle_a"] >> max_angle_a;
    fs["threshold"] >> threshold;
    rm_auto_aim::Detector::LightParams l_params = {
        .min_ratio = min_ratio,
        .max_ratio = max_ratio,
        .max_angle = max_angle_l};

    rm_auto_aim::Detector::ArmorParams a_params = {
        .min_light_ratio = 0.7,
        .min_small_center_distance = min_small_center_distance,
        .max_small_center_distance = max_small_center_distance,
        .min_large_center_distance = min_large_center_distance,
        .max_large_center_distance = max_large_center_distance,
        .max_angle = max_angle_a};

    auto detector = std::make_unique<rm_auto_aim::Detector>(binary_thres, detect_color, l_params, a_params);

    // Init classifier
    std::string pkg_path = "/home/csy/Desktop/RM/infantry_9_28";
    auto model_path = pkg_path + "/model/mlp.onnx";
    auto label_path = pkg_path + "/model/label.txt";
    // double threshold = 0.7;
    std::vector<std::string> ignore_classes =
        std::vector<std::string>{"negative"};
    detector->classifier =
        std::make_unique<rm_auto_aim::NumberClassifier>(model_path, label_path, threshold, ignore_classes);

    return detector;
}

cv::Point3f AimAuto::findTarget(Translator &ts, double pz, double pyaw, double &real_time)
{
    double flight_time, react_time;

    std::vector<cv::Point3f> armors;
    real_time = flight_time = 0;
    react_time = 0.7;
    if (ts.message.status % 5 != 2)
    {
        pz = 0;
        std::cout << "et:" << ts.message.status << std::endl;
        do
        {
            armors.clear();
            real_time = flight_time;
            double x_c = tracker->target_state(0);
            double y_c = tracker->target_state(1);
            double pred_x_c = x_c + tracker->target_state(4) * real_time;
            double pred_y_c = y_c + tracker->target_state(5) * real_time;
            double pred_yaw = tracker->target_state(3) + tracker->target_state(7) * real_time;
            double pred_x_a = pred_x_c - this->r * cos(pred_yaw);
            double pred_y_a = pred_y_c + this->r * sin(pred_yaw);
            armors.push_back(cv::Point3f(pred_x_a, pred_y_a, tracker->target_state(2)));
            armors.push_back(cv::Point3f(2 * pred_x_c - pred_x_a, 2 * pred_y_c - pred_y_a, tracker->target_state(2)));
            armors.push_back(cv::Point3f(pred_x_c + pred_y_c - pred_y_a, pred_y_c - pred_x_c + pred_x_a, tracker->last_z));
            armors.push_back(cv::Point3f(pred_x_c - pred_y_c + pred_y_a, pred_y_c + pred_x_c - pred_x_a, tracker->last_z));
            sort(armors.begin(), armors.end(), [](const cv::Point3f &la, const cv::Point3f &lb)
                 { return sqrt(la.x * la.x + la.y * la.y) < sqrt(lb.x * lb.x + lb.y * lb.y); });
            flight_time = timer.getFlightTime(armors[0], (double)ts.message.bullet_v / 10);
        } while (abs(flight_time - real_time) > 0.0001);
    }
    else
    {
        std::cout << "et" << std::endl;
        pz = 150;
        do
        {
            armors.clear();
            real_time = flight_time + react_time;
            double x_c = tracker->target_state(0);
            double y_c = tracker->target_state(1);
            double pred_x_c = x_c + tracker->target_state(4) / 10 * real_time;
            double pred_y_c = y_c + tracker->target_state(5) / 10 * real_time;
            double pred_yaw = tracker->target_state(3) + tracker->target_state(7) * real_time + pyaw;
            double pred_x_a = pred_x_c - this->r * cos(pred_yaw);
            double pred_y_a = pred_y_c + this->r * sin(pred_yaw);
            armors.push_back(cv::Point3f(pred_x_a, pred_y_a, tracker->target_state(2)));
            armors.push_back(cv::Point3f(pred_x_c + cos(2 * M_PI / 3) * (pred_x_a - pred_x_c) - sin(2 * M_PI / 3) * (pred_y_a - pred_y_c), pred_y_c + sin(2 * M_PI / 3) * (pred_x_a - pred_x_c) + cos(2 * M_PI / 3) * (pred_y_a - pred_y_c), tracker->target_state(2)));
            armors.push_back(cv::Point3f(pred_x_c + cos(2 * M_PI / 3) * (pred_x_a - pred_x_c) + sin(2 * M_PI / 3) * (pred_y_a - pred_y_c), pred_y_c - sin(2 * M_PI / 3) * (pred_x_a - pred_x_c) + cos(2 * M_PI / 3) * (pred_y_a - pred_y_c), tracker->target_state(2)));
            sort(armors.begin(), armors.end(), [](const cv::Point3f &la, const cv::Point3f &lb)
                 { return sqrt(la.x * la.x + la.y * la.y) < sqrt(lb.x * lb.x + lb.y * lb.y); });
            flight_time = timer.getFlightTime(armors[0], (double)ts.message.bullet_v / 10);
        } while (abs(flight_time + react_time - real_time) > 0.001);
    }
    this->flt=flight_time;
    return armors[0];


}

void AimAuto::storeMessage(cv::Point3f target, Translator &ts)
{
    ts.message.x_a = target.x * 1000;
    ts.message.y_a = target.y * 1000;
    ts.message.z_a = target.z * 1000;
    ts.message.x_c = tracker->target_state(0) * 1000;
    ts.message.y_c = tracker->target_state(1) * 1000;
    ts.message.z_c = tracker->target_state(2) * 1000;
    ts.message.yaw_a = armors_msg.armors[0].yaw;
    ts.message.vx_c = tracker->target_state(4) * 1000 / 1.33;
    ts.message.vy_c = tracker->target_state(5) * 1000 / 1.33;
    ts.message.vz_c = tracker->target_state(6) * 1000 / 1.33;
    ts.message.vyaw_a = tracker->target_state(7) / 1.33;
    ts.message.armor_flag = tracker->tracked_armor.type;

    
}

void AimAuto::convertPoint(Translator &ts, cv::Point3f target)
{
    Eigen::MatrixXd m_pitch(3, 3);
    Eigen::MatrixXd m_yaw(3, 3);
    m_yaw << cos(ts.message.yaw), -sin(ts.message.yaw), 0, sin(ts.message.yaw), cos(ts.message.yaw), 0, 0, 0, 1;
    m_pitch << cos(ts.message.pitch), 0, sin(ts.message.pitch), 0, 1, 0, -sin(ts.message.pitch), 0, cos(ts.message.pitch);
    Eigen::Vector3d temp;
    temp = Eigen::Vector3d(target.x, target.y, target.z);
    Eigen::Vector3d position = m_pitch.inverse() * m_yaw.inverse() * temp;
    ts.message.x_a = -position(1) * 1000;
    ts.message.y_a = -(position(2) * 1000);
    ts.message.z_a = position(0) * 1000;
}

void AimAuto::fireControl(Translator &ts, cv::Mat &src)
{
    Eigen::MatrixXd _K_(3, 3);
    _K_ << (float)gp->fx, 0, (float)gp->cx, 0, (float)gp->fy, (float)gp->cy, 0, 0, 1;
    // Eigen::Vector3d CameraAxis=Eigen::Vector3d(ts.message.x_a,ts.message.y_a,ts.message.z_a);
    Eigen::Vector3d CameraAxis = Eigen::Vector3d(ts.message.x_a, ts.message.y_a, ts.message.z_a);
    Eigen::Vector3d PictureAxis = _K_ * CameraAxis;
    int PictureX = (int)(PictureAxis[0] / PictureAxis[2]);
    int PictureY = (int)(PictureAxis[1] / PictureAxis[2]);
    cv::Point predictPoint(PictureX, PictureY);
    // if (abs(PictureX - (src.size[1] / 2)) < 10)
    std::cout << "PC:" << PictureX - (src.size[1] / 2) << std::endl;
    preshow.recvStatus(src, 1000 *this->time,  predictPoint, this->flt);
    std::cout<<"flight time:"<<this->flt<<std::endl;
    if ((PictureX - (src.size[1] / 2)) < 20 && (PictureX - (src.size[1] / 2)) > -10)
    {
        ts.message.armor_flag = 1;
        std::cout << "hit !!" << std::endl;
        cv::circle(src, predictPoint, 10, cv::Scalar(0, 255, 0), 2);
    }
    else
        ts.message.armor_flag = 0;
}

void AimAuto::showDist(std::deque<Armor> &tar_list, cv::Mat &src)
{
    if (tar_list.size() != 0)
    {
        cv::putText(src, "X:" + std::to_string(tar_list[0].center.x), cv::Point(30, 60),
                    cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
        cv::putText(src, "Y:" + std::to_string(tar_list[0].center.y), cv::Point(30, 90),
                    cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
        cv::putText(src, "Z:" + std::to_string(tar_list[0].center.z), cv::Point(30, 120),
                    cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
        cv::putText(src, "type:" + std::to_string(tar_list[0].type), cv::Point(30, 150),
                    cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
    }
    else
    {
        cv::putText(src, "X: NOT FOUND", cv::Point(30, 60),
                    cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
        cv::putText(src, "Y: NOT FOUND", cv::Point(30, 90),
                    cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
        cv::putText(src, "Z: NOT FOUND", cv::Point(30, 120),
                    cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 100, 0));
    }
}

bool AimAuto::updateTracker(Translator &ts, cv::Mat &src)
{
    if (tracker->tracker_state == Tracker::LOST)
    {
        if (armors_msg.armors[0].type == 1 || gp->isBigArmor[armors_msg.armors[0].type])
        {
            this->r = 0.35;
        }
        else
        {
            this->r = 0.28;
        }
        if (ts.message.status % 5 == 2)
        {
            this->r = 0.2765;
            if (armors_msg.armors[0].type != 0)
                return false;
        }
        tracker->init(armors_msg, this->r);
    }
    else
    {
        // Set dt
        dt = this->time - this->last_time;
        dt /= 1000;
        if (armors_msg.armors[0].type == 1 || gp->isBigArmor[armors_msg.armors[0].type])
        {
            this->r = 0.35;
        }
        else
        {
            this->r = 0.28;
        }
        if (ts.message.status % 5 == 2)
        {
            this->r = 0.2765;
            if (armors_msg.armors[0].type != 0)
                return false;
        }
        // Update state
        tracker->update(armors_msg, src, this->r);
        last_time = this->time;
    }
    return true;
}