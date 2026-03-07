#ifndef PATH_TRACKER_H
#define PATH_TRACKER_H

#include <utility>

class Odometry;

class PathTracker
{
    double setpointX_ = 0; // [m]
    double setpointY_ = 0; // [m]

    double command_v_ = 0; // [m/s]
    double command_w_ = 0; // [rad/s]

    // regulation params
    double k_rho_ = 3;
    double k_alpha_ = 8;
    double k_beta_ = -1.5;

    // run info
    bool is_running_ = false;

    const double V_MAX = 0.4;     // [m/s]
    const double W_MAX = 3.14159; // [rad/s]

public:
    void setSetpoint(double x, double y);
    double getSetpointX() { return setpointX_; }
    double getSetpointY() { return setpointY_; }
    void update(Odometry odom);
    std::pair<double, double> getCommand();
    bool isRunning();
    void start();
    void stop();
};

#endif // PATH_TRACKER_H
