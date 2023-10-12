#include "tracker.hpp"

// STD

static inline double normalize_angle(double angle)
{
    const double result = fmod(angle + M_PI, 2.0 * M_PI);
    if (result <= 0.0)
        return result + M_PI;
    return result - M_PI;
}
double shortest_angular_distance(double from, double to)
{
    return normalize_angle(to - from);
}
Tracker::Tracker(
    double max_match_distance, int tracking_threshold, int lost_threshold,
    Eigen::DiagonalMatrix<double, 8> q, Eigen::DiagonalMatrix<double, 4> r)
    : tracker_state(LOST),
      tracked_id(-1),
      target_state(Eigen::VectorXd::Zero(8)),
      max_match_distance_(max_match_distance),
      tracking_threshold_(tracking_threshold),
      lost_threshold_(lost_threshold)
{
}

void Tracker::init(const Armors &armors_msg, double r)
{
    this->r = r;
    if (armors_msg.armors.empty())
    {
        return;
    }
    tracked_armor = armors_msg.armors[0];
    initEKF(tracked_armor);
    tracked_id = tracked_armor.type;
    tracker_state = DETECTING;
}

void Tracker::update(const Armors &armors_msg, cv::Mat &src, double r)
{
    this->r = r;
    // KF predict
    Eigen::VectorXd ekf_prediction = ekf.predict();

    bool matched = false;
    // Use KF prediction as default target state if no matched armor is found
    target_state = ekf_prediction;

    if (!armors_msg.armors.empty())
    {
        double min_position_diff = DBL_MAX;
        auto predicted_position = getArmorPositionFromState(ekf_prediction);
        for (const auto &armor : armors_msg.armors)
        {
            auto p = armor.position;
            Eigen::Vector3d position_vec(p(0), p(1), p(2));
            // Difference of the current armor position and tracked armor's predicted position
            double position_diff = (predicted_position - position_vec).norm();
            if (position_diff < min_position_diff)
            {
                min_position_diff = position_diff;
                tracked_armor = armor;
            }
        }
        cv::line(src, tracked_armor.apex[0], tracked_armor.apex[1], cv::Scalar(0, 0, 255), 3);
        cv::line(src, tracked_armor.apex[1], tracked_armor.apex[2], cv::Scalar(0, 0, 255), 3);
        cv::line(src, tracked_armor.apex[2], tracked_armor.apex[3], cv::Scalar(0, 0, 255), 3);
        cv::line(src, tracked_armor.apex[3], tracked_armor.apex[0], cv::Scalar(0, 0, 255), 3);
        if (abs(tracked_armor.yaw - ekf_prediction(3)) < 0.2)
        // if (min_position_diff < max_match_distance_)
        {
            // Matching armor found
            matched = true;
            auto p = tracked_armor.position;
            // Update EKF
            double measured_yaw = tracked_armor.yaw;
            Eigen::Vector4d z(p(0), p(1), p(2), measured_yaw);
            target_state = ekf.update(z);
        }
        else
        {
            // Check if there is same id armor in current frame
            for (const auto &armor : armors_msg.armors)
            {
                if (armor.type == tracked_id)
                {
                    // Armor jump happens
                    matched = true;
                    tracked_armor = armor;
                    handleArmorJump(tracked_armor);
                    break;
                }
            }
        }
    }
    if (this->r == 0.2765)
    {
        if (abs(target_state(7)) > 0.3)
            target_state(7) = 0.4 * target_state(7) / abs(target_state(7));
        else
            target_state(7) = 0.2 * target_state(7) / abs(target_state(7));
        target_state(4) = 0;
        target_state(5) = 0;
        target_state(6) = 0;
        ekf.setState(target_state);
    }
    // Tracking state machine
    if (tracker_state == DETECTING)
    {
        if (matched)
        {
            detect_count_++;
            if (detect_count_ > tracking_threshold_)
            {
                detect_count_ = 0;
                tracker_state = TRACKING;
            }
        }
        else
        {
            detect_count_ = 0;
            tracker_state = LOST;
        }
    }
    else if (tracker_state == TRACKING)
    {
        if (!matched)
        {
            tracker_state = TEMP_LOST;
            lost_count_++;
        }
    }
    else if (tracker_state == TEMP_LOST)
    {
        std::cout << "temp lost" << std::endl;
        if (!matched)
        {
            lost_count_++;
            if (lost_count_ > lost_threshold_)
            {
                lost_count_ = 0;
                tracker_state = LOST;
            }
        }
        else
        {
            tracker_state = TRACKING;
            lost_count_ = 0;
        }
    }
}
void Tracker::initEKF(const Armor &a)
{
    double xa = a.position(0);
    double ya = a.position(1);
    double za = a.position(2);
    double yaw = a.yaw;
    // Set initial position at 0.2m behind the target
    target_state = Eigen::VectorXd::Zero(8);
    double xc = xa + r * cos(yaw);
    double yc = ya - r * sin(yaw);
    double zc = za;
    last_z = zc;
    target_state << xc, yc, zc, yaw, 0, 0, 0, 0;
    ekf.setState(target_state);
    // std::cout << "init again" << std::endl;
}

void Tracker::handleArmorJump(const Armor &a)
{
    std::cout << "jumped!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    double last_yaw = target_state(3);
    if (abs(a.yaw - last_yaw) > 0.2)
    {
        std::cout << "swap" << std::endl;
        last_z = target_state(2);
        target_state(2) = a.position(2);
        target_state(3) = a.yaw;
    }
    auto p = a.position;
    Eigen::Vector3d current_p(p(0), p(1), p(2));
    Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);
    // 误差过大，需要从新重置
    if ((current_p - infer_p).norm() > max_match_distance_)
    {
        std::cout << "reset" << std::endl;
        target_state(0) = p(0) + r * cos(a.yaw);
        target_state(1) = p(1) - r * sin(a.yaw);
        target_state(4) = 0;
        target_state(5) = 0;
        target_state(6) = 0;
    }
    ekf.setState(target_state);
    // auto x = a.position;
    // // Update EKF
    // double measured_yaw = a.yaw;
    // Eigen::Vector4d z(x(0), x(1), x(2), measured_yaw);
    // target_state = ekf.update(z);
}

Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd &x)
{
    // Calculate predicted position of the current armor
    double xc = x(0), yc = x(1), zc = x(2);
    double yaw = x(3);
    double xa = xc - r * cos(yaw);
    double ya = yc + r * sin(yaw);
    return Eigen::Vector3d(xa, ya, zc);
}