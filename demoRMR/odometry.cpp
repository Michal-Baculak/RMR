#include "odometry.h"
#include <iostream>
#include <math.h>
#include <stdint.h>

Odometry::Odometry()
    : _posX(0)
    , _posY(0)
    , _rot(0)
    , _rotPrev(0)
{}

void Odometry::update(TKobukiData robotData)
{
    int16_t deltaLeft = static_cast<int16_t>(static_cast<uint16_t>(robotData.EncoderLeft) - static_cast<uint16_t>(_encoderLeftPrev));
    int16_t deltaRight = static_cast<int16_t>(static_cast<uint16_t>(robotData.EncoderRight) - static_cast<uint16_t>(_encoderRightPrev));
    double deltaRot = (robotData.GyroAngle - _gyroAnglePrev) * PI / 18000.0;

    _encoderLeftPrev = robotData.EncoderLeft;
    _encoderRightPrev = robotData.EncoderRight;
    _gyroAnglePrev = robotData.GyroAngle;

    unsigned short deltaT = robotData.synctimestamp - _timestampPrev;
    _timestampPrev = robotData.synctimestamp;

    double l_l = deltaLeft * _tickToMeter;
    double l_r = deltaRight * _tickToMeter;
    // double velLeft = l_l / deltaT;
    // double velRight = l_r / deltaT;
    double deltaDist =  (l_l + l_r)/ 2.0;

    _rot += deltaRot;

    while (_rot > PI)
        _rot -= 2 * PI;
    while (_rot < -PI)
        _rot += 2 * PI;

    //double myRot = robotData.GyroAngle/100.0;

    // TODO: implement linear and arc motion, do not use gyro in arc motion
    if(std::abs(l_l - l_r) < 1e-6)
    {
        _posX += deltaDist * cos(_rot);
        _posY += deltaDist * sin(_rot);
    } else {
        _posX += (_wheelBase * (l_r + l_l))/(2 * (l_r - l_l)) * (sin(_rot) - sin(_rotPrev));
        _posY -= (_wheelBase * (l_r + l_l))/(2 * (l_r - l_l)) * (cos(_rot) - cos(_rotPrev));
    }

    _rotPrev = _rot;

    std::cout << "rot with delta: " << _rot << ", rot direct from gyro: " << myRot << std::endl;
    //std::cout << "velLeft: " << velLeft << ", velRight: " << velRight << std::endl;
    //std::cout << "Odometry: x = " << _posX << ", y = " << _posY << ", rot = " << _rot << std::endl;
}

void Odometry::init(TKobukiData initData)
{
    _encoderLeftPrev = initData.EncoderLeft;
    _encoderRightPrev = initData.EncoderRight;
    _timestampPrev = initData.synctimestamp;
    _gyroAnglePrev = initData.GyroAngle;

    _isInitialized = true;
}
