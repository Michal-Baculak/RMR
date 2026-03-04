#include "odometry.h"
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

    std::cout << "Odometry: x = " << _posX << ", y = " << _posY << ", rot = " << _rot << ", rotPrev = " << _rotPrev << std::endl;

    _rotPrev = _rot;

    if(deltaT > 0)
    {
        _omega = deltaRot / deltaT;
        _v = deltaDist / deltaT;
    }

    std::cout << robotData.synctimestamp << ", " << deltaT << std::endl;
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
