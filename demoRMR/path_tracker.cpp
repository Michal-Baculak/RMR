#include "path_tracker.h"
#include "algorithm"
#include "odometry.h"

void PathTracker::setSetpoint(double x, double y)
{
    setpointX_ = x;
    setpointY_ = y;
}

void PathTracker::regulate(double rho, double alpha, double beta)
{
    double v = k_rho_ * rho;
    double w = k_alpha_ * alpha + k_beta_ * beta;

    std::cout << "***************Position Controller***************" << std::endl;
    std::cout << "P controller suggests v = " << v << ", w = " << w << std::endl;

    if (rho < POSITION_EPSILON) {
        stop();
        std::cout << "Position error " << rho << "is under tolerance: " << POSITION_EPSILON
                  << ", stopping,..." << std::endl;
        return;
    }

    // determine path curvature - we want to preserve that and only adjust speed
    double k = w / v; // v is never zero here
    std::cout << "curvature of desired path is k = " << k << std::endl;

    // apply velocity profiling when far from target
    if (rho > REGULATION_ZONE_DIST) {
        // profile velocity
        double profiled_vel = getProfiledVelocity(rho, REGULATION_ZONE_DIST);
        if (profiled_vel > v)
            v = profiled_vel;
    } else
        std::cout << "Near target, applying precise position regulation..." << std::endl;

    // calculate suggested acceleration
    double dt = 0.01; // no idea how to calculate this accurately...
    double a = (v - command_v_) / dt;
    double epsilon = (w - command_w_) / dt;

    // clamp the acceleration and velocity
    double a_clamped = std::clamp(a, -ACCELERATION_MAX, ACCELERATION_MAX);
    double epsilon_clamped = std::clamp(epsilon,
                                        -ANGULAR_ACCELERATION_MAX,
                                        ANGULAR_ACCELERATION_MAX);

    // new speed command
    v = command_v_ + dt * a_clamped; // v is still not zero here!
    w = command_w_ + dt * epsilon_clamped;

    std::cout << "smooth next velocity is v = " << v << ", w = " << w << std::endl;

    // clamped speed commands
    command_v_ = std::clamp(v, -V_MAX, V_MAX);
    command_w_ = std::clamp(w, -W_MAX, W_MAX);

    std::cout << "after clamping: v = " << command_v_ << ", w  = " << command_w_ << std::endl;

    // curvature adjust commands
    if (k == 0)
        // this means w = 0, we dont need cross adjusting
        return;

    std::cout << "curvature after clamping speeds is " << command_w_ / command_v_ << std::endl;
    if (abs(command_w_ / command_v_) < abs(k)) {
        // v is relatively higher, make it match the curvature:
        // k = w/v
        // v = w/k
        std::cout << "Adjusting velocity to match curvature..." << std::endl;
        command_v_ = command_w_ / k;
    }

    if (abs(command_w_ / command_v_) > abs(k)) {
        // w is relatively higher, lower it
        // k = w/v
        // w = k*v
        std::cout << "Adjusting angular velocity to match curvature..." << std::endl;
        command_w_ = command_v_ * k;
    }
}

void PathTracker::update(Odometry odom)
{
    double theta = odom.getRot();
    double dx = setpointX_ - odom.getX();
    double dy = setpointY_ - odom.getY();
    double rho = sqrt(dx * dx + dy * dy);
    double alpha = wrap(-theta + atan2(dy, dx));
    double beta = -theta - alpha;

    regulate(rho, alpha, beta);
}

void PathTracker::updateVFH(Odometry odom, double safe_heading)
{
    double rho = (POSITION_EPSILON + REGULATION_ZONE_DIST) / 2;
    double theta = odom.getRot();
    double alpha = wrap(safe_heading - theta);
    double beta = 0;
    regulate(rho, alpha, beta);
}

double PathTracker::getProfiledVelocity(double dist, double regulationZoneDist)
{
    // assume linearly increaing speed -> max speed -> linearly decreasing speed up to regulationZoneDist
    // we obtain this velocity by answering the question: how fast can we go, so that we can safely stop withing the distance of (dist-regulationZoneDist)?
    double s = dist - regulationZoneDist;

    if (s < 0)
        return 0;
    double v = sqrt(2 * s * ACCELERATION_MAX);
    return std::min(v, V_MAX); // clamp to V_MAX
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
    command_v_ = 0;
    command_w_ = 0;
}

bool PathTracker::isRunning()
{
    return is_running_;
}

double PathTracker::wrap(double x)
{
    x = fmod(x + PI, 2 * PI);
    if (x < 0)
        x += 2 * PI;
    return x - PI;
}

// void PathTracker::updateVFH(Odometry odom, double safeTarget)
// {
//     double theta = odom.getRot();
//     double dx = setpointX_ - odom.getX();
//     double dy = setpointY_ - odom.getY();
//     double rho = std::sqrt(dx * dx + dy * dy);

//     double alpha = wrap(-theta + safeTarget);
//     double beta = 0;

//     double v = k_rho_ * rho;
//     double w = k_alpha_ * alpha + k_beta_ * beta;

//     // if (rho < POSITION_EPSILON) {
//     //     stop();
//     //     return;
//     // }

//     // double k = w / v;

//     // if (rho > REGULATION_ZONE_DIST) {
//     //     // profile velocity
//     //     double profiled_vel = getProfiledVelocity(rho, REGULATION_ZONE_DIST);
//     //     if (profiled_vel > v)
//     //         v = profiled_vel;
//     // } else
//     //     std::cout << "Near target, applying precise position regulation..." << std::endl;

//     // double dt = 0.01;
//     // double a = (v - command_v_) / dt;
//     // double epsilon = (w - command_w_) / dt;

//     // double a_clamped = std::clamp((v - command_v_) / dt, -ACCELERATION_MAX, ACCELERATION_MAX);
//     // double epsilon_clamped = std::clamp((w - command_w_) / dt, -ANGULAR_ACCELERATION_MAX, ANGULAR_ACCELERATION_MAX);

//     // command_v_ = std::clamp(command_v_ + dt * a_clamped, -V_MAX, V_MAX);
//     // command_w_ = std::clamp(command_w_ + dt * epsilon_clamped, -W_MAX, W_MAX);

//     // // curvature adjust commands
//     // if (k == 0)
//     //     // this means w = 0, we dont need cross adjusting
//     //     return;

//     // std::cout << "curvature after clamping speeds is " << command_w_ / command_v_ << std::endl;
//     // if (abs(command_w_ / command_v_) < abs(k)) {
//     //     // v is relatively higher, make it match the curvature:
//     //     // k = w/v
//     //     // v = w/k
//     //     std::cout << "Adjusting velocity to match curvature..." << std::endl;
//     //     command_v_ = command_w_ / k;
//     // }

//     // if (abs(command_w_ / command_v_) > abs(k)) {
//     //     // w is relatively higher, lower it
//     //     // k = w/v
//     //     // w = k*v
//     //     std::cout << "Adjusting angular velocity to match curvature..." << std::endl;
//     //     command_w_ = command_v_ * k;
//     // }

//     double lookahead = std::min(rho, LOOKAHEAD_DIST);

//     double localTargetX = odom.getX() + lookahead * std::cos(safeTarget);
//     double localTargetY = odom.getY() + lookahead * std::sin(safeTarget);

//     double ldx   = localTargetX - odom.getX();
//     double ldy   = localTargetY - odom.getY();
//     double lrho  = std::sqrt(ldx * ldx + ldy * ldy); // ≈ lookahead

//     if (rho > REGULATION_ZONE_DIST) {
//         double profiled = getProfiledVelocity(rho, REGULATION_ZONE_DIST);
//         if (profiled > v) v = profiled;
//     }

//     const double dt = 0.01; //

//     double a       = std::clamp((v - command_v_) / dt, -ACCELERATION_MAX,       ACCELERATION_MAX);
//     double epsilon = std::clamp((w - command_w_) / dt, -ANGULAR_ACCELERATION_MAX, ANGULAR_ACCELERATION_MAX);

//     v = command_v_ + dt * a;
//     w = command_w_ + dt * epsilon;

//     command_v_ = std::clamp(v, -V_MAX, V_MAX);
//     command_w_ = std::clamp(w, -W_MAX, W_MAX);

//     if (std::abs(command_v_) > 1e-6) {
//         double k_desired = w / v;

//         if (std::abs(k_desired) > 1e-9) {
//             double k_actual = command_w_ / command_v_;

//             if (std::abs(k_actual) < std::abs(k_desired)) {

//                 command_v_ = command_w_ / k_desired;
//                 command_v_ = std::clamp(command_v_, -V_MAX, V_MAX);
//             } else if (std::abs(k_actual) > std::abs(k_desired)) {

//                 command_w_ = command_v_ * k_desired;
//                 command_w_ = std::clamp(command_w_, -W_MAX, W_MAX);
//             }
//         }
//     }
}
