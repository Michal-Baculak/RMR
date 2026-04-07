#include "mapper.h"
#include "librobot/rplidar.h"
#include "odometry.h"
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

void Mapper::init()
{
    // we consider the worst case scenario, that we are placed on the edge of the map
    _mid_point = static_cast<size_t>(MAP_MAX_SIZE / MAP_CELL_SIZE);
    _map_size = 1 + 2 * _mid_point;
    _map_cv = cv::Mat::zeros((int) _map_size, (int) _map_size, CV_16UC1);
}

void Mapper::init(cv::Mat map)
{
    if (map.rows != map.cols) {
        std::cerr << "Imported map is not square! Aborting..." << std::endl;
        return;
    }
    std::cout << "Initialising map with cv::Mat" << std::endl;
    _map_size = map.rows;
    _mid_point = _map_size / 2;

    cv::Mat mask = map != 0;
    _map_cv = cv::Mat::zeros(map.rows, map.cols, CV_16UC1);
    _map_cv.setTo(HITS_TO_REGISTER, mask);
}

bool Mapper::isInitialized()
{
    // abuse or feature? When midpoint is 0, we can hardly consider the mapper initialized!
    return _mid_point != 0;
}

uint16_t &Mapper::getAt(int x, int y)
{
    return _map_cv.at<uint16_t>(_mid_point + x, _mid_point + y);
}

uint16_t &Mapper::getAt(double x, double y)
{
    int x_int = round(x / MAP_CELL_SIZE);
    int y_int = round(y / MAP_CELL_SIZE);
    return getAt(x_int, y_int);
}

cv::Point Mapper::pointToMapIndex(Point real_position) const
{
    int x_int = round(real_position.x / MAP_CELL_SIZE);
    int y_int = round(real_position.y / MAP_CELL_SIZE);

    cv::Point map_index{(int) (x_int + _mid_point), (int) (y_int + _mid_point)};
    return map_index;
}

Point Mapper::mapIndexToPoint(cv::Point map_coords) const
{
    int x_int = map_coords.x - _mid_point;
    int y_int = map_coords.y - _mid_point;

    Point out{x_int * MAP_CELL_SIZE, y_int * MAP_CELL_SIZE};
    return out;
}

inline void Mapper::registerPoint(Point p)
{
    uint16_t &map_val = getAt(p.x, p.y);
    registerPoint(map_val);
}

inline void Mapper::registerPoint(uint16_t &map_point)
{
    if (map_point < HITS_TO_REGISTER)
        map_point += 1;
    if (map_point > HITS_TO_REGISTER)
        map_point = HITS_TO_REGISTER;
}

void Mapper::update(Odometry odom, const std::vector<LaserData> &laserData)
{
    for (auto &beam : laserData) {
        if (beam.scanDistance / 1000.0 < LIDAR_MIN_DIST)
            continue;
        Point point_coords = odom.laserToPoint(beam);
        registerPoint(point_coords);
    }
}
void Mapper::update(const std::vector<XYQPoint> &parsedPoints)
{
    for (auto &laser_pt : parsedPoints) {
        uint16_t &map_val = getAt(laser_pt.p.x, laser_pt.p.y);
        registerPoint(map_val);
    }
}

size_t Mapper::getMapSize()
{
    return _map_size;
}

uint16_t Mapper::getMapElement(size_t ix, size_t iy) const
{
    return _map_cv.at<uint16_t>((int) ix, (int) iy);
}

cv::Mat Mapper::getMapRaw() const
{
    return _map_cv.clone();
}

cv::Mat Mapper::getMapFiltered() const
{
    // get elements, that are "stable"
    return (_map_cv == ID_OBSTACLE);
}

//***************************** Path planning *****************************
void Mapper::clearPlan()
{
    // clear even inflated obstacles - map might be updated
    cv::Mat mask = (_map_cv >= ID_OBSTACLE_INFLATED);
    _map_cv.setTo(0, mask);
    isPlanned = false;
}

std::vector<cv::Point> Mapper::getElementsWithinDistance(cv::Point from, int distance) const
{
    std::vector<cv::Point> output;
    int side_len = 2 * distance + 1;
    output.reserve(side_len * side_len);
    for (int i = -distance; i <= distance; i++) {
        int x = from.x + i;
        if (x < 0 || x > _map_size - 1)
            continue;
        for (int j = -distance; j <= distance; j++) {
            int y = from.y + j;
            if (y < 0 || y > _map_size - 1)
                continue;
            output.push_back({x, y});
        }
    }
    return output;
}

void Mapper::setTo(uint16_t val, const std::vector<cv::Point> &indices)
{
    for (const auto &index : indices) {
        _map_cv.at<uint16_t>(index) = val;
    }
}

void Mapper::inflateObstacles(double radius)
{
    int expansion_count = std::ceil(radius / MAP_CELL_SIZE);
    cv::Mat obstacles_mask = (_map_cv == ID_OBSTACLE);

    std::vector<cv::Point> obstacle_points;
    cv::findNonZero(obstacles_mask, obstacle_points);

    // go through each obstacle and inflate
    for (const cv::Point &obstacle_coord : obstacle_points) {
        std::vector<cv::Point> near_coords = getElementsWithinDistance(obstacle_coord,
                                                                       expansion_count);
        setTo(ID_OBSTACLE_INFLATED, near_coords);
    }

    // obstacle inflation changes the obstacles too, revert that
    _map_cv.setTo(ID_OBSTACLE, obstacles_mask);
}

bool Mapper::floodFillIteration(std::vector<cv::Point> &idxs_to_check,
                                std::vector<cv::Point> &idxs_updated,
                                uint16_t level)
{
    for (cv::Point pt_main : idxs_to_check) {
        std::vector<cv::Point> neighbours = getElementsWithinDistance(pt_main, 1);
        for (cv::Point neighbour : neighbours) {
            uint16_t &val = _map_cv.at<uint16_t>(neighbour);
            if (val < ID_OBSTACLE) {
                // map point is empty - update distance from start
                val = level;
                idxs_updated.push_back(neighbour);
            }
            if (val == ID_GOAL_POS) {
                return true;
            }
        }
    }
}

bool Mapper::floodFill(cv::Point start)
{
    std::vector<cv::Point> pts_updated;
    std::vector<cv::Point> pts_to_check;
    pts_to_check.push_back(start);
    uint16_t level = ID_PLANNING_RANGE_START;
    bool finished = false;
    do {
        pts_updated.clear();
        finished = floodFillIteration(pts_to_check, pts_updated, ID_PLANNING_RANGE_START);
        pts_to_check = pts_updated;
        if (!finished && level == 0xFFFF)
            return false;
        ++level;
    } while (finished == false);
    return true;
}

// std::vector<cv::Point> getKeypoints(cv::Point start)
// {
//     // start is for convenience, so we dont have to look for it
//     std::vector<cv::Point> key_indices;
//     bool goal_reached = false;
//     // TODO
// }

void Mapper::plan(Point from, Point to)
{
    if (isPlanned) {
        // clear map
        clearPlan();
    }
    inflateObstacles(0.2);
    cv::Mat inflated_map = (_map_cv == ID_OBSTACLE_INFLATED);
    cv::imshow("inflated obstacles", inflated_map);

    getAt(from.x, from.y) = ID_START_POS;
    getAt(to.x, to.y) = ID_GOAL_POS;

    bool success = floodFill(pointToMapIndex(from));
    if (!success)
        return;

    // TODO...
    // find key waypoints
    isPlanned = success;
}
