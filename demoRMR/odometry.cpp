#include "odometry.h"
#include <iostream>
#include <math.h>
#include <stdint.h>

Odometry::Odometry()
    : _posX(0)
    , _posY(0)
    , _rot(0)
    , _vel(0)
{}

void Odometry::update(TKobukiData robotData)
{
    int16_t deltaLeft = static_cast<int16_t>(static_cast<uint16_t>(robotData.EncoderLeft) - static_cast<uint16_t>(_encoderLeftPrev));
    int16_t deltaRight = static_cast<int16_t>(static_cast<uint16_t>(robotData.EncoderRight) - static_cast<uint16_t>(_encoderRightPrev));
    _encoderLeftPrev = robotData.EncoderLeft;
    _encoderRightPrev = robotData.EncoderRight;

    // std::cout << "deltaLeft: " << deltaLeft << ", deltaRight: " << deltaRight << std::endl;

    unsigned short deltaT = robotData.synctimestamp - _timestampPrev;
    _timestampPrev = robotData.synctimestamp;

    double l_l = deltaLeft * _tickToMeter;
    double l_r = deltaRight * _tickToMeter;
    double velLeft = l_l / deltaT;
    double velRight = l_r / deltaT;
    double deltaDist =  (l_l + l_r)/ 2.0;
    _rot = (robotData.GyroAngle / 100.0) * PI / 180.0;
    // _rot += (l_r - l_l) / _wheelBase;
    // while (_rot > PI)
    //     _rot -= 2 * PI;
    // while (_rot < -PI)
    //     _rot += 2 * PI;

    // TODO: sensor rot fusion
    _posX += deltaDist * cos(_rot);
    _posY += deltaDist * sin(_rot);

    std::cout << "velLeft: " << velLeft << ", velRight: " << velRight << std::endl;
    //std::cout << "Odometry: x = " << _posX << ", y = " << _posY << ", rot = " << _rot << std::endl;
}

void Odometry::init(TKobukiData initData)
{
    _encoderLeftPrev = initData.EncoderLeft;
    _encoderRightPrev = initData.EncoderRight;
    _timestampPrev = initData.synctimestamp;

    _isInitialized = true;
}
