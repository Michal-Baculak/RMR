#ifndef MAPPER_H
#define MAPPER_H

#include "librobot/CKobuki.h"
#include "librobot/rplidar.h"
#include <opencv2/core/mat.hpp>
#include <vector>

class Odometry;
struct XYQPoint;
struct Point;

class Mapper
{
    const double MAP_MAX_SIZE = 7.0;   // [m] - max size of map in one direction
    const double MAP_CELL_SIZE = 0.05; // [m] - size of one cell of map
    const uint16_t HITS_TO_REGISTER
        = 20; // number of LiDAR point hits necessary to consider an obstacle on a map
    const uint16_t ID_OBSTACLE = HITS_TO_REGISTER;
    const uint16_t ID_OBSTACLE_INFLATED = HITS_TO_REGISTER + 1;
    const uint16_t ID_START_POS = HITS_TO_REGISTER + 2;
    const uint16_t ID_GOAL_POS = HITS_TO_REGISTER + 3;
    const uint16_t ID_PLANNING_RANGE_START = HITS_TO_REGISTER + 4;
    const double WEAR_OFF_COEFF
        = 0.8; //after each iteration, this is the amount to which the registration count will reduce
    const double LIDAR_MIN_DIST = 0.1; // [m] - minimal valid LiDAR reading point

    cv::Mat _map_cv;
    size_t _mid_point = 0;
    size_t _map_size = 0;
    bool isPlanned = false;

private:
    uint16_t &getAt(int x, int y);
    uint16_t &getAt(double x, double y);
    std::vector<cv::Point> getElementsWithinDistance(cv::Point from, int distance) const;
    void inflateObstacles(double radius);
    void setTo(uint16_t val, const std::vector<cv::Point> &indices);
    bool floodFillIteration(std::vector<cv::Point> &idxs_to_check,
                            std::vector<cv::Point> &idxs_updated,
                            uint16_t level);
    bool floodFill(cv::Point start);

public:
    size_t getMapSize();

    bool isInitialized();
    void init();
    void init(cv::Mat map);
    void update(Odometry odom, const std::vector<LaserData> &laserData);
    void update(const std::vector<XYQPoint> &parsedPoints);
    inline void registerPoint(uint16_t &map_point);
    inline void registerPoint(Point p);
    uint16_t getMapElement(size_t ix, size_t iy) const;
    cv::Mat getMapRaw() const;
    cv::Mat getMapFiltered() const;
    cv::Point pointToMapIndex(Point real_position) const;
    Point mapIndexToPoint(cv::Point map_coords) const;
    void plan(Point from, Point to);
    void clearPlan();
};

#endif // MAPPER_H
