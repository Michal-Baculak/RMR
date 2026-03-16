#ifndef MAPPER_H
#define MAPPER_H

#include "librobot/CKobuki.h"
#include "librobot/rplidar.h"
#include <vector>

class Odometry;

class Mapper
{
    const double MAP_MAX_SIZE = 7.0;   // [m] - max size of map in one direction
    const double MAP_CELL_SIZE = 0.05; // [m] - size of one cell of map
    const uint16_t HITS_TO_REGISTER = 10;
    const double WEAR_OFF_COEFF
        = 0.8; //after each iteration, this is the amount to which the registration count will reduce

    std::vector<std::vector<uint16_t>> _map;
    size_t _mid_point = 0;
    size_t _map_size = 0;

private:
    uint16_t &getAt(int x, int y);
    uint16_t &getAt(double x, double y);

public:
    size_t getMapSize();

    bool isInitialized();
    void init();
    void update(Odometry odom, const std::vector<LaserData> &laserData);
    uint16_t getMapElement(size_t ix, size_t iy);
};

#endif // MAPPER_H
