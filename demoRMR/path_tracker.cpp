#include "path_tracker.h"
#include "algorithm"
#include "odometry.h"

// void PathTracker::pushSetpoint()

void PathTracker::regulate(double rho, double alpha, double beta)
{
    double v = k_rho_ * rho;
    double w = k_alpha_ * alpha + k_beta_ * beta;

    std::cout << "***************Position Controller***************" << std::endl;
    std::cout << "P controller suggests v = " << v << ", w = " << w << std::endl;

    // if (rho < POSITION_EPSILON) {
    //     stop();
    //     std::cout << "Position error " << rho << "is under tolerance: " << POSITION_EPSILON
    //               << ", stopping,..." << std::endl;
    //     return;
    // }

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
    double dt = SAMPLING_PERIOD;
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
    double v_clamped = std::clamp(v, -V_MAX, V_MAX);
    double w_clamped = std::clamp(w, -W_MAX, W_MAX);

    std::cout << "after clamping: v = " << v_clamped << ", w  = " << w_clamped << std::endl;

    // curvature adjust commands
    if (k == 0)
        // this means w = 0, we dont need cross adjusting
        return;

    double command_v_new = v_clamped;
    double command_w_new = w_clamped;
    std::cout << "curvature after clamping speeds is " << w_clamped / v_clamped << std::endl;
    if (abs(w_clamped / v_clamped) < abs(k)) {
        // v is relatively higher, make it match the curvature:
        // k = w/v
        // v = w/k
        std::cout << "Adjusting velocity to match curvature..." << std::endl;
        double v_optimal = w_clamped / k;
        double a_optimal = (v_optimal - command_v_) / SAMPLING_PERIOD;
        double a_limited = std::clamp(a_optimal, -BRAKING_MAX, ACCELERATION_MAX);
        command_v_new = command_v_ + a_limited * SAMPLING_PERIOD;
    }

    if (abs(w_clamped / v_clamped) > abs(k)) {
        // w is relatively higher, lower it? No, we can allow it

        // k = w/v
        // w = k*v
        //std::cout << "Adjusting angular velocity to match curvature..." << std::endl;
        command_w_new = v_clamped * k;
    }
    command_w_ = command_w_new;
    command_v_ = command_v_new;
}

double PathTracker::getDistToSetpoint(Odometry odom)
{
    double dx = getSetpointX() - odom.getX();
    double dy = getSetpointY() - odom.getY();
    return sqrt(dx * dx + dy * dy);
}

void PathTracker::update(Odometry odom)
{
    double theta = odom.getRot();
    double dx = getSetpointX() - odom.getX();
    double dy = getSetpointY() - odom.getY();
    double rho = sqrt(dx * dx + dy * dy);
    double alpha = wrap(-theta + atan2(dy, dx));
    double beta = -theta - alpha;

    if (rho < POSITION_EPSILON) {
        stop();
        std::cout << "Position error " << rho << "is under tolerance: " << POSITION_EPSILON
                  << ", stopping,..." << std::endl;
        return;
    }

    regulate(rho, alpha, beta);
}

void PathTracker::updateVFH(Odometry odom, double safe_heading)
{
    //double rho = 0.1 * POSITION_EPSILON + 0.9 * REGULATION_ZONE_DIST;
    double rho = getDistToSetpoint(odom);

    // if we reached checkpoint
    while (setpoints.size() > 1 && rho < POSITION_EPSILON_DYNAMIC) {
        std::cout << "Checkpoint along trajectory reached " << std::endl;
        //pop the checkpoint
        setpoints.pop_front();
        rho = getDistToSetpoint(odom);
    }

    double theta = odom.getRot();
    double alpha = wrap(safe_heading - theta);
    double beta = 0;

    if (rho < POSITION_EPSILON) {
        std::cout << "Reactive navigation reached destination " << rho << std::endl;
        stop();
        return;
    }

    double heading_diff_abs = std::abs(alpha);
    if (heading_diff_abs > PI / 4.0) {
        rho = 0.3 * POSITION_EPSILON + 0.7 * REGULATION_ZONE_DIST;
    } else {
        rho = rho * (1.0 - (heading_diff_abs / (PI / 4.0)));
    }
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
    setpoints.clear();
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

void PathTracker::setTrajectory(const std::vector<Point> &trajectory)
{
    setpoints = std::deque<Point>(trajectory.begin(), trajectory.end());
}

void PathTracker::setSetpoint(double x, double y)
{
    setpoints.push_front({x, y});
    // setpointX_ = x;
    // setpointY_ = y;
}

void PathTracker::setGoalSetpoint(double x, double y)
{
    setpoints.push_back({x, y});
}

double PathTracker::getSetpointX()
{
    return setpoints.at(0).x;
}

double PathTracker::getSetpointY()
{
    return setpoints.at(0).y;
}
