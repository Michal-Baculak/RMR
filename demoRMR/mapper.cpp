#include "mapper.h"
#include "librobot/rplidar.h"
#include "odometry.h"
#include <cmath>

void Mapper::init()
{
    // we consider the worst case scenario, that we are placed on the edge of the map
    _mid_point = static_cast<size_t>(MAP_MAX_SIZE / MAP_CELL_SIZE);
    _map_size = 1 + 2 * _mid_point;
    for (size_t i = 0; i < _map_size; ++i) {
        _map.push_back(std::vector<uint16_t>(_map_size, 0));
    }
}

bool Mapper::isInitialized()
{
    // abuse or feature? When midpoint is 0, we can hardly consider the mapper initialized!
    return _mid_point != 0;
}

uint16_t &Mapper::getAt(int x, int y)
{
    return _map.at(_mid_point + x).at(_mid_point + y);
}

uint16_t &Mapper::getAt(double x, double y)
{
    int x_int = round(x / MAP_CELL_SIZE);
    int y_int = round(y / MAP_CELL_SIZE);
    return getAt(x_int, y_int);
}

void Mapper::update(Odometry odom, const std::vector<LaserData> &laserData)
{
    for (auto &beam : laserData) {
        if (beam.scanDistance / 1000.0 < LIDAR_MIN_DIST)
            continue;
        Point point_coords = odom.laserToPoint(beam);
        uint16_t &map_val = getAt(point_coords.x, point_coords.y);
        if (map_val < 100)
            map_val += 5;
        if (map_val > 100)
            map_val = 100;
    }
}

size_t Mapper::getMapSize()
{
    return _map_size;
}

uint16_t Mapper::getMapElement(size_t ix, size_t iy)
{
    return _map.at(ix).at(iy);
}
