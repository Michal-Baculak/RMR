#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "librobot/CKobuki.h"

class Odometry
{
private:
    double _posX;
    double _posY;
    double _rot;

    unsigned short _encoderLeftPrev;
    unsigned short _encoderRightPrev;
    unsigned short _timestampPrev;

public:
    void update(TKobukiData robotData);
    void resetPos(double x, double y);
};

#endif // ODOMETRY_H
