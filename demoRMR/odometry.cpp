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

    // TODO: Work in progress
    // double deltaDist = (deltaLeft + deltaRight) / 2.0 * ;
    // _posX += deltaDist * cos(robotData.GyroAngle)

    std::cout << "Odometry update called encoder data: " << robotData.EncoderLeft << std::endl;
}
