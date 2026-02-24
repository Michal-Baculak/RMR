#include "odometry.h"
#include <iostream>
#include <math.h>
#include <stdint.h>

Odometry::Odometry()
    : _posX(0)
    , _posY(0)
    , _rot(0)
{}

void Odometry::update(TKobukiData robotData)
{
    int16_t deltaLeft = static_cast<int16_t>(static_cast<uint16_t>(robotData.EncoderLeft) - static_cast<uint16_t>(_encoderLeftPrev));
    int16_t deltaRight = static_cast<int16_t>(static_cast<uint16_t>(robotData.EncoderRight) - static_cast<uint16_t>(_encoderRightPrev));
    _encoderLeftPrev = robotData.EncoderLeft;
    _encoderRightPrev = robotData.EncoderRight;

    std::cout << "deltaLeft: " << deltaLeft << ", deltaRight: " << deltaRight << std::endl;

    unsigned short deltaT = robotData.timestamp - _timestampPrev;
    _timestampPrev = robotData.timestamp;

    double l_l = deltaLeft * _tickToMeter;
    double l_r = deltaRight * _tickToMeter;
    double deltaDist =  (l_l + l_r)/ 2.0;
    _rot += (l_r - l_l) / _wheelBase;
    while (_rot > PI)
        _rot -= 2 * PI;
    while (_rot < -PI)
        _rot += 2 * PI;

    // TODO: sensor rot fusion
    _posX += deltaDist * cos(_rot);
    _posY += deltaDist * sin(_rot);

    std::cout << "Odometry: x = " << _posX << ", y = " << _posY << ", rot = " << _rot << std::endl;
}

void Odometry::init(TKobukiData initData)
{
    _encoderLeftPrev = initData.EncoderLeft;
    _encoderRightPrev = initData.EncoderRight;
    _timestampPrev = initData.timestamp;

    _isInitialized = true;
}
