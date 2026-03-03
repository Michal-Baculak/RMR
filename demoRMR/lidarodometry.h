#ifndef LIDARODOMETRY_H
#define LIDARODOMETRY_H

#include <librobot/librobot.h>
#include <vector>

struct Point2D
{
    double x;
    double y;
};

class LidarOdometry
{
private:
    std::vector<Point2D> _prevScan;
    bool _isInitialized = false;

    double _posX;
    double _posY;
    double _rot;

public:
    LidarOdometry();

    void update(std::vector<LaserData>& scan);
    bool isInitialized() { return _isInitialized; }
    void init(std::vector<LaserData>& scan);

    double getX() { return _posX; }
    double getY() { return _posY; }
    double getRot() {return _rot; }

    std::vector<Point2D> convertScan(std::vector<LaserData>& scan);
    void icp(std::vector<Point2D>& source,
             std::vector<Point2D>& target,
             double& dx, double& dy, double& drot);

};

#endif // LIDARODOMETRY_H
