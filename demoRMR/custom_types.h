#ifndef CUSTOM_TYPES_H
#define CUSTOM_TYPES_H

#include <cstdint>

struct Pose
{
    double x;
    double y;
    double phi;
};

struct Point
{
    double x;
    double y;
};

struct XYQPoint
{
    Point p;
    int scanQuality;
    uint32_t timestamp;
};

#endif // CUSTOM_TYPES_H
