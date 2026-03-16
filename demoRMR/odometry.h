#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "librobot/CKobuki.h"
#include "librobot/rplidar.h"
#include <utility>
#include <vector>

struct Pose;
struct Point;
struct XYQPoint;

class Odometry
{
private:
    double _posX;
    double _posY;
    double _rot;
    double _rotPrev;
    double _omega;
    double _v;

    unsigned short _encoderLeftPrev;
    unsigned short _encoderRightPrev;
    signed short _gyroAnglePrev;
    uint32_t _timestampPrev;

    // We will use these magic constants
    static constexpr double _wheelBase = 0.230; //razvor kolies v metroch
    static constexpr double _tickToMeter = 0.000085292090497737556558;

    bool _isInitialized = false;

    std::vector<std::pair<uint32_t, Pose>> _poseStack; // positions are saved latest
    const size_t POSE_STACK_MAX_SIZE = 7;              // Position -> 40Hz, LiDAR -> ??Hz

public:
    void update(TKobukiData robotData);
    void resetPos(double x, double y);
    Odometry();
    void init(TKobukiData initData);
    bool isInitialized() { return _isInitialized; }
    double getX() {return _posX;}
    double getY() {return _posY;}
    double getRot() {return _rot;}
    double getOmega() {return _omega;}
    double getV() {return _v;}
    void compensateLidarScan(std::vector<LaserData> &laserData, std::vector<XYQPoint> &parsedPoints);
    void compensateLidarScan(std::vector<LaserData> &laserData);
    Pose interpolatePosition(Pose p1, Pose p2, uint32_t t1, uint32_t t2, uint32_t t);
    Pose extrapolatePosition(Pose p0, double v, double w, double t);
    Pose getCurrentPoseEstimate(uint32_t currentTimestamp);
    Point laserToPoint(LaserData laser);
    Point laserToPoint(Pose observer, LaserData laser);
};

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

#endif // ODOMETRY_H
