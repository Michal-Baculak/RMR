#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "custom_types.h"
#include "librobot/CKobuki.h"
#include "librobot/rplidar.h"
#include <deque>
#include <utility>
#include <vector>

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
    const double LIDAR_MIN_DIST = 0.1; // [m] - minimal valid LiDAR reading point

    bool _isInitialized = false;

    std::deque<std::pair<uint32_t, Pose>> _poseStack; // positions are saved latest (use deque)
    const size_t POSE_STACK_MAX_SIZE = 15; // Position -> 40Hz, LiDAR -> 8Hz => need at least 5

public:
    void update(TKobukiData robotData);
    void resetPos(double x, double y);
    Odometry();
    void init(TKobukiData initData);
    bool isInitialized() { return _isInitialized; }
    double getX() const { return _posX; }
    double getY() const { return _posY; }
    double getRot() const { return _rot; }
    double getOmega() const { return _omega; }
    double getV() const { return _v; }
    void compensateLidarScan(std::vector<LaserData> &laserData, std::vector<XYQPoint> &parsedPoints);
    void compensateLidarScan(std::vector<LaserData> &laserData);
    Pose interpolatePosition(Pose p1, Pose p2, uint32_t t1, uint32_t t2, uint32_t t);
    Pose extrapolatePosition(Pose p0, double v, double w, double t);
    Pose getCurrentPoseEstimate(uint32_t currentTimestamp);
    Point laserToPoint(LaserData laser);
    Point laserToPoint(Pose observer, LaserData laser);
};

#endif // ODOMETRY_H
