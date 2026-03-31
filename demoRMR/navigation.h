#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <librobot/rplidar.h>
#include "librobot/CKobuki.h"

class Odometry;

class Navigation
{
public:
    Navigation();
    double update(double rX, double rY, double rPhi, double targetX, double targetY, const std::vector<LaserData> &laserData, double currentV, double currentW);
    void printLaserData(const std::vector<LaserData>& laser);
    const std::vector<int>& getLastMHist() const { return _last_mHist; }
private:
    const int NUM_SECTORS = 120; // number of sectors
    const double SIGMA = 3.0; // discrimination ability
    const double RS = 0.1; // [m] - enlargement of obstacle
    const double R_MIN_STATIC = 0.30;
    const int S_MAX = 5;
    const double RADIUS = 0.17;

    // threshold values
    double _t_low = 10;
    double _t_high = 20;

    std::vector<int> _prev_binary_hist;
    int _prev_dir = 0;

    // constants for magnitude calculation
    const double _a = 1.5;
    double _b = 0.4;

    // weights for cost function
    const double _mu1 = 5.0;
    const double _mu2 = 2.0;
    const double _mu3 = 2.0;

    int circularDist(int k1, int k2) const;

    std::vector<int> _last_mHist;
};

#endif // NAVIGATION_H
