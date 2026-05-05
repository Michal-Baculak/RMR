#ifndef PATH_TRACKER_H
#define PATH_TRACKER_H

#include "custom_types.h"
#include <deque>
#include <utility>
#include <vector>

class Odometry;

class PathTracker
{
    // double setpointX_ = 0; // [m]
    // double setpointY_ = 0; // [m]

    double command_v_ = 0; // [m/s]
    double command_w_ = 0; // [rad/s]

    // regulation params
    double k_rho_ = 3;    // Default: 3
    double k_alpha_ = 16; // Default: 8
    double k_beta_ = 0;   // Default: -1.5 -> set to zero for position-only regulation

    // run info
    bool is_running_ = false;
    std::deque<Point> setpoints;

    const double V_MAX = 0.4;         // [m/s]
    const double W_MAX = 3.14159 / 5; // [rad/s]
    const double POSITION_EPSILON
        = 0.015; // [m] if position error falls in this range, position is considered reached and controller stops
    const double POSITION_EPSILON_DYNAMIC
        = 0.2; // [m] if position error falls in this range, controller sets next setpoint
    const double REGULATION_ZONE_DIST
        = 0.1; // [m] if the distance from setpoint falls withing this bound, velocity is no longer profiled and is instead being set by regulator
    // it should hold that POSITION_EPSILON_DNYMIC > REGULATION_ZONE_DIST > POSITION_EPSILON > 0
    const double LOOKAHEAD_DIST = 0.30;               // added for VFH, consider removing
    const double ACCELERATION_MAX = 0.5 * 1;          // [m/s^2]
    const double BRAKING_MAX = 2 * ACCELERATION_MAX;
    const double ANGULAR_ACCELERATION_MAX = 3 * 3.14; // [rad/s^2]
    const double SAMPLING_PERIOD
        = 0.025; //[s] - we are setting this to a constant, to solve for sudden jumps if frames are missed

public:
    void setSetpoint(double x, double y);
    const std::deque<Point> &getSetpoints() const { return setpoints; }
    double getSetpointX();
    double getSetpointY();

    void setGoalSetpoint(double x, double y);
    void setTrajectory(const std::vector<Point> &trajectory);

    void update(Odometry odom);
    void updateVFH(Odometry odom, double safe_heading);
    double getProfiledVelocity(double dist, double regulationZoneDist);
    std::pair<double, double> getCommand();
    bool isRunning();
    void start();
    void stop();
    static double wrap(double angle);
    void brake();
    void transformSetpoints(Pose old_pose, Pose new_pose);

private:
    void regulate(double rho, double alpha, double beta);
    double getDistToSetpoint(Odometry odom);
};

#endif // PATH_TRACKER_H
