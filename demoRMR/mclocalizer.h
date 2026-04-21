#ifndef MCLOCALIZER_H
#define MCLOCALIZER_H

#include <librobot/rplidar.h>
#include <random>

#include "custom_types.h"
#include "opencv2/opencv.hpp"

struct Particle
{
    Pose pose;
    double weight;
};

class mclocalizer
{
public:
    mclocalizer();
    void init(const cv::Mat &binObstacleMap, double cellSize, int numParticles);
    void updateMotion(double dx, double dy, double dPhi);
    void updateWeights(const std::vector<LaserData> &laserData);
    void resample();

private:
    std::mt19937 _rng;
};



#endif // MCLOCALIZER_H
