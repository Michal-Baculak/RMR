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

    // We will use these magic constants
    static constexpr double _wheelBase = 0.230; //razvor kolies v metroch
    static constexpr double _tickToMeter = 0.000085292090497737556558;

    bool _isInitialized = false;

public:
    void update(TKobukiData robotData);
    void resetPos(double x, double y);
    Odometry();
    void init(TKobukiData initData);
    bool isInitialized() { return _isInitialized; }
};

#endif // ODOMETRY_H
