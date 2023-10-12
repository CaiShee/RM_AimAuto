#ifndef ARMOR_PROCESSOR__TRACKER_HPP_
#define ARMOR_PROCESSOR__TRACKER_HPP_

// Eigen
#include <Eigen/Eigen>

// STD
#include "KalmanFilter.hpp"
#include "globalParam.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <cfloat>
#include <deque>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
class Tracker
{
public:
    Tracker(
        double max_match_distance, int tracking_threshold, int lost_threshold,
        Eigen::DiagonalMatrix<double, 8> q, Eigen::DiagonalMatrix<double, 4> r);

    void init(const Armors &armors_msg, double r);
    void update(const Armors &armors_msg, cv::Mat &src, double r);
    enum State
    {
        LOST,
        DETECTING,
        TRACKING,
        TEMP_LOST,
    } tracker_state;
    ExtendedKalmanFilter ekf;
    Armor tracked_armor;
    int tracked_id;
    Eigen::VectorXd target_state;

    double last_z, last_r, r;

private:
    void initEKF(const Armor &a);

    void handleArmorJump(const Armor &a);

    Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd &x);

    double max_match_distance_;

    int tracking_threshold_;
    int lost_threshold_;

    int detect_count_;
    int lost_count_;
};

#endif // ARMOR_PROCESSOR__TRACKER_HPP_
