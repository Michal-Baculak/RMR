#ifndef MAPPER_H
#define MAPPER_H

#include "librobot/CKobuki.h"
#include "librobot/rplidar.h"
#include <vector>

class Odometry;

class Mapper
{
public:
    void init();
    void update(Odometry odom, const std::vector<LaserData> &laserData);
};

#endif // MAPPER_H
