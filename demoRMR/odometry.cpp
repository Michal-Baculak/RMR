#include "odometry.h"
#include <iostream>

void Odometry::update(TKobukiData robotData)
{
    unsigned short deltaLeft = robotData.EncoderLeft - _encoderLeftPrev;
    unsigned short deltaRight = robotData.EncoderRight - _encoderRightPrev;
    _encoderLeftPrev = robotData.EncoderLeft;
    _encoderRightPrev = robotData.EncoderRight;

    unsigned short deltaT = robotData.timestamp - _timestampPrev;
    _timestampPrev = robotData.timestamp;

    double l_l = deltaLeft * _tickToMeter;
    double l_r = deltaRight * _tickToMeter;
    double deltaDist =  (l_l + l_r)/ 2.0;
    _posX += deltaDist * cos(robotData.GyroAngle);
    _posY += deltaDist * sin(robotData.GyroAngle);
    _rot += (l_l - l_r)/_wheelBase;

    // TODO: Work in progress
    // double deltaDist = (deltaLeft + deltaRight) / 2.0 * ;
    // _posX += deltaDist * cos(robotData.GyroAngle)



    //std::cout << "Odometry update called encoder data: " << robotData.EncoderLeft << std::endl;
    std::cout << "Odometry: x = " << _posX << ", y = " << _posY << ", rot = " << _rot << std::endl;
}
