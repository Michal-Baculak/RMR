#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <cmath>
#include <optional>
#include <librobot/rplidar.h>
#include "librobot/CKobuki.h"
#include "algorithm"


class Odometry;

class Navigation
{
public:
    Navigation();
    // double update(double rX, double rY, double rPhi, double targetX, double targetY, const std::vector<LaserData> &laserData, double currentV, double currentW);
    std::optional<double> update(const std::vector<LaserData> &laserData, double rPhi, double currentV, double currentW, double targetAngle);
    const std::vector<int>& getLastMHist() const { return _last_mHist; }
    bool isDirWithinCurrentSector(double dir, double robot_rot) const;

private:
    const int NUM_SECTORS = 120;
    const double SIGMA = 3.0; // rozlisovacia schopnost (pocet stupnov na sektor)
    const double DS = 0.05; // [m] - bezpecna vzdialenost od prekazky, ktoru ma robot dodrzat
    const double R_MIN_STATIC = 0.0;
    const int S_MAX = 20;
    const double RADIUS = 0.17; // polomer robota
    const double WIN_SIZE = 0.5;

    // threshold values
    const double _t_low = 10;
    const double _t_high = 20;

    std::vector<int> _prev_binary_hist;
    int _prev_dir = 0;

    // constants for magnitude calculation
    const double _a = 18.0;
    const double _b = 7.0;

    // weights for cost function
    const double _mu1 = 3.5;
    const double _mu2 = 1.5;
    const double _mu3 = 3.0;

    const double _valley_edge = S_MAX/6;

    int circularDist(int k1, int k2) const;
    std::vector<int> findCandidateSectors(const std::vector<int> &mHist, int k_target) const;
    std::vector<double> calcPHist(const std::vector<LaserData>  &laserData, double rPhi);
    int angleToSector(double angle_deg) const;
    double sectorToAngle(int k) const;
    int selectBestSector(const std::vector<int> &candidates,int k_target, int k_robot_heading) const;
    double sectorToSafeHeading(int sector, double robot_angle_deg) const;
    std::vector<int> _last_mHist;
};

#endif // NAVIGATION_H
