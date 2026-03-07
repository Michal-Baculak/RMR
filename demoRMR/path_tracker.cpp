#include "path_tracker.h"
#include "algorithm"
#include "odometry.h"

void PathTracker::setSetpoint(double x, double y)
{
    setpointX_ = x;
    setpointY_ = y;
}

void PathTracker::update(Odometry odom)
{
    double theta = odom.getRot();
    double dx = setpointX_ - odom.getX();
    double dy = setpointY_ - odom.getY();
    double rho = sqrt(dx * dx + dy * dy);
    double alpha = -theta + atan2(dy, dx);
    double beta = -theta - alpha;

    double v = k_rho_ * rho;
    double w = k_alpha_ * alpha + k_beta_ * beta;

    //calculated commands
    command_v_ = std::clamp(v, -V_MAX, V_MAX);
    command_w_ = std::clamp(w, -W_MAX, W_MAX);

    std::cout << "Calculated command: v=" << command_v_ << ", w=" << command_w_ << std::endl;
}

std::pair<double, double> PathTracker::getCommand()
{
    return std::pair<double, double>(command_v_, command_w_);
}

void PathTracker::start()
{
    is_running_ = true;
}

void PathTracker::stop()
{
    is_running_ = false;
}

bool PathTracker::isRunning()
{
    return is_running_;
}
