#include "odometry.h"
#include "utility.h"
#include <iostream>
#include <math.h>
#include <stdint.h>

Odometry::Odometry()
    : _posX(0)
    , _posY(0)
    , _omega(0)
    , _v(0)
{}

void Odometry::update(TKobukiData robotData)
{
    int16_t deltaLeft = static_cast<int16_t>(static_cast<uint16_t>(robotData.EncoderLeft) - static_cast<uint16_t>(_encoderLeftPrev));
    int16_t deltaRight = static_cast<int16_t>(static_cast<uint16_t>(robotData.EncoderRight) - static_cast<uint16_t>(_encoderRightPrev));
    double deltaRot = (robotData.GyroAngle - _gyroAnglePrev) * PI / 18000.0; // rad
    deltaRot = utility::wrap(deltaRot);

    _encoderLeftPrev = robotData.EncoderLeft;
    _encoderRightPrev = robotData.EncoderRight;
    _gyroAnglePrev = robotData.GyroAngle; // deg

    double deltaT = (robotData.synctimestamp - _timestampPrev)/1000000.0;
    _timestampPrev = robotData.synctimestamp;

    double l_l = deltaLeft * _tickToMeter;
    double l_r = deltaRight * _tickToMeter;
    double deltaDist =  (l_l + l_r)/ 2.0;
    //double deltaRot = (l_r - l_l)/_wheelBase; // diff between rotation angles from odometry

    _rot += deltaRot; // rad

    while (_rot > PI)
        _rot -= 2 * PI;
    while (_rot < -PI)
        _rot += 2 * PI;

    _posX += deltaDist * cos(_rot);
    _posY += deltaDist * sin(_rot);

    // Cannot use this because of small gyro resolution. If we want to use this, it's necessary
    // to use deltaRot from wheel odometry

    // if(std::abs(l_r - l_l) < 1e-6)
    // {
    //     _posX += deltaDist * cos(_rot);
    //     _posY += deltaDist * sin(_rot);
    // }
    // else
    // {
    //     double factor = (_wheelBase*(l_r + l_l))/(2*(l_r - l_l));
    //     _posX += factor*(sin(_rot) - sin(_rotPrev));
    //     _posY -= factor*(cos(_rot) - cos(_rotPrev));
    // }

    // std::cout << "Odometry: x = " << _posX << ", y = " << _posY << ", rot = " << _rot << ", rotPrev = " << _rotPrev << std::endl;

    _rotPrev = _rot;

    if(deltaT > 0)
    {
        _omega = deltaRot / deltaT;
        _v = deltaDist / deltaT;
    }

    // keep stack of positions for LiDAR movement compensation
    _poseStack.push_back({robotData.synctimestamp, {_posX, _posY, _rot}});
    if (_poseStack.size() > POSE_STACK_MAX_SIZE)
        _poseStack.erase(_poseStack.begin());

    // std::cout << robotData.synctimestamp << ", " << deltaT << std::endl;
}

void Odometry::init(TKobukiData initData)
{
    _encoderLeftPrev = initData.EncoderLeft;
    _encoderRightPrev = initData.EncoderRight;
    _timestampPrev = initData.synctimestamp;

    _rot = initData.GyroAngle * PI / 18000; // rad
    _rotPrev = _rot; // rad
    _gyroAnglePrev = initData.GyroAngle; // deg

    _isInitialized = true;

    std::cout << "Angle init rot: " << _rot << ", rotPrev: " << _rotPrev << ", gyroPrev: " << _gyroAnglePrev << std::endl;;
}

void Odometry::compensateLidarScan(std::vector<LaserData> &laserData,
                                   std::vector<XYQPoint> &parsedPoints)
{
    parsedPoints.clear();
    for (auto &laserBeam : laserData) {
        if (laserBeam.scanDistance < LIDAR_MIN_DIST)
            continue;
        // dead zone
        if (laserBeam.scanDistance > 0.6 * 1000 && laserBeam.scanDistance < 0.75 * 1000)
            continue;
        // find the two saved poses, betweeen which the beam was executed
        uint32_t min_diff = 0 - 1;
        size_t min_idx = 0;
        bool is_valid = false;
        for (size_t i = 0, stack_size = _poseStack.size(); i < stack_size; ++i) {
            uint32_t pose_ts = _poseStack.at(i).first;
            if (laserBeam.timestamp < pose_ts)
                continue;
            is_valid = true;
            uint32_t diff = laserBeam.timestamp - pose_ts;
            if (diff >= min_diff)
                continue;
            min_diff = diff;
            min_idx = i;
        }

        if (!is_valid) {
            std::cout << "laser beam is not valid, removing..." << std::endl;
            laserBeam.scanDistance = 0;
            continue;
        }

        // min_idx is the index of last saved pose before the laser returned
        // we need one more pose to interpolate.
        Pose laser_origin;
        if (min_idx >= _poseStack.size() - 1) {
            // // calculate pose based on velocity
            // auto last_pose_pair = _poseStack.at(_poseStack.size() - 1);
            // uint32_t last_timestamp = last_pose_pair.first;
            // Pose last_pose = last_pose_pair.second;
            // uint32_t deltaT = laserBeam.timestamp - last_timestamp;
            // double dt = deltaT / 1000000.0; // -ish? (TODO)
            // laser_origin = extrapolatePosition(last_pose, _v, _omega, dt);
            // std::cerr << "Extrapolating LiDAR data (this should not happen!)" << std::endl;
            // std::cerr << "This beam timestamp: " << laserBeam.timestamp << std::endl;
            // for (size_t i = 0; i < _poseStack.size(); ++i) {
            //     std::cerr << "timestamp at " << i << " position is " << _poseStack.at(i).first
            //               << std::endl;
            // }
            std::cerr << "Interpolation slip, skipping..." << std::endl;
            continue;
        } else {
            // interpolate pose
            size_t next_idx = min_idx + 1;
            // std::cout << "interpolating laser origin position between indices " << min_idx
            //           << " and " << next_idx << std::endl;
            laser_origin = interpolatePosition(_poseStack.at(min_idx).second,
                                               _poseStack.at(next_idx).second,
                                               _poseStack.at(min_idx).first,
                                               _poseStack.at(next_idx).first,
                                               laserBeam.timestamp);
        }

        // calculate the laser point position w.r.t. the origin pose
        double laser_angle = laser_origin.phi - laserBeam.scanAngle * PI / 180;
        double laser_x = laser_origin.x + laserBeam.scanDistance * cos(laser_angle) / 1000;
        double laser_y = laser_origin.y + laserBeam.scanDistance * sin(laser_angle) / 1000;

        // recalculate w.r.t the current position
        double beam_dx = laser_x - _posX;
        double beam_dy = laser_y - _posY;
        double beam_dist = sqrt(beam_dx * beam_dx + beam_dy * beam_dy);
        double beam_angle = atan2(beam_dy, beam_dx) - _rot;

        // modify beam
        laserBeam.scanAngle = -beam_angle * 180 / PI;
        laserBeam.scanDistance = beam_dist * 1000;
        //std::cout << "min index for this laser beam: " << min_idx << std::endl;

        // save parsed (XY) points
        parsedPoints.push_back({{laser_x, laser_y}, laserBeam.scanQuality, laserBeam.timestamp});
    }
}

void Odometry::compensateLidarScan(std::vector<LaserData> &laserData)
{
    std::vector<XYQPoint> empty;
    compensateLidarScan(laserData, empty);
}

Pose Odometry::interpolatePosition(Pose p1, Pose p2, uint32_t t1, uint32_t t2, uint32_t t)
{
    Pose output;
    double progress = static_cast<double>(t - t1) / (t2 - t1);
    output.x = p1.x + (p2.x - p1.x) * progress;
    output.y = p1.y + (p2.y - p1.y) * progress;

    double d_phi = utility::wrap(p2.phi - p1.phi);
    output.phi = p1.phi + d_phi * progress;
    output.phi = utility::wrap(output.phi);
    return output;
}

Pose Odometry::extrapolatePosition(Pose p0, double v, double w, double t)
{
    Pose output;
    double d_phi = w * t;

    if (w == 0) {
        // only linear motion
        output.phi = p0.phi;
        output.x = p0.x + v * t * cos(p0.phi);
        output.y = p0.y + v * t * sin(p0.phi);
        return output;
    }

    // w is non-zero here
    double r = v / w;

    // this math is taken from prednaska (kudos)
    double ds = 2 * r * sin(d_phi / 2);

    double dx = ds * cos(d_phi / 2 + p0.phi);
    double dy = ds * sin(d_phi / 2 + p0.phi);

    output.x = p0.x + dx;
    output.y = p0.y + dy;
    output.phi = utility::wrap(p0.phi + d_phi);

    return output;
}
Pose Odometry::getCurrentPoseEstimate(uint32_t currentTimestamp)
{
    auto last_pose_pair = _poseStack.at(_poseStack.size() - 1);
    uint32_t deltaT = currentTimestamp - last_pose_pair.first;
    double dt = deltaT / 1000000;
    return extrapolatePosition(last_pose_pair.second, _v, _omega, dt);
}

Point Odometry::laserToPoint(LaserData laser)
{
    return laserToPoint({_posX, _posY, _rot}, laser);
}

Point Odometry::laserToPoint(Pose observer, LaserData laser)
{
    Point output;
    double angle = observer.phi - laser.scanAngle * PI / 180;
    output.x = observer.x + laser.scanDistance * cos(angle) / 1000;
    output.y = observer.y + laser.scanDistance * sin(angle) / 1000;
    return output;
}

void Odometry::setPose(Pose pose, TKobukiData data)
{
    _poseStack.clear();
    _posX = pose.x;
    _posY = pose.y;
    _rot = pose.phi;
    _rotPrev = _rot;
    init(data);
}
