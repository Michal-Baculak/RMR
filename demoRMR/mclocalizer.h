#ifndef MCLOCALIZER_H
#define MCLOCALIZER_H

#include <librobot/rplidar.h>
#include <random>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>

#include "custom_types.h"
#include "opencv2/opencv.hpp"
#include "librobot/CKobuki.h"
#include "utility.h"

class mclocalizer
{
public:
    mclocalizer();
    void init(const cv::Mat &binObstacleMap, double cellSize, int numParticles);
    void updateMotion(double dx, double dy, double dPhi);
    void updateWeights(const std::vector<LaserData> &laserData);
    void resample();
    void setMotionNoise(double a1, double a2, double a3, double a4);
    Pose getBestPose() const;
    double distToNearestObstacle(double x, double y) const;
    const std::vector<Particle> &getParticles() const {return _particles; }
    cv::Mat getDistanceField() const { return _distanceField; }
    bool isInitialized() const {return _initialized; }
    cv::Mat getVisualization() const;

    bool isLocalized() const;

private:
    cv::Point poseToMapIndex(double x, double y) const;
    bool isFree(double x, double y) const;
    Pose sampleRandomFreePose();
    void createDistanceField();
private:
    std::mt19937 _rng;
    cv::Mat _obstacleMap;
    cv::Mat _distanceField;
    cv::Mat _insideMask;
    double _cellSize = 0.0;
    int _midPoint = 0;
    std::vector<Particle> _particles;
    bool _initialized = false;

    cv::Point _bboxMin{0, 0};
    cv::Point _bboxMax{0, 0};
    bool _bboxComputed = false;
    std::vector<cv::Point> _validFreeCells;

    double _a1 = 0.05; //rot noise from rotation
    double _a2 = 0.05; //rot noise from translation
    double _a3 = 0.10; //trans noise from translation
    double _a4 = 0.05; //trans noise from rotation

    int MAX_ATTEMPTS = 1000;
    double TRANS_EPS = 1e-3;

    int LASER_STRIDE = 5;
    double LIDAR_MIN_M = 0.1;
    double LIDAR_MAX_M = 5.0;
    double WEIGHT_EPS = 1e-6;

    double _injectionRatio = 0.05;

    double _locStdXY = 0.1;
    double _locStdPhi = 0.1;
};



#endif // MCLOCALIZER_H
