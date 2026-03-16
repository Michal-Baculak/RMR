#define _USE_MATH_DEFINES

#include "utility.h"
#include "algorithm"
#include <cmath>

namespace utility {

// TODO: replace odometry wrap function with this one
double wrap(double x)
{
    x = fmod(x + M_PI, 2 * M_PI);
    if (x < 0)
        x += 2 * M_PI;
    return x - M_PI;
}
} // namespace utility
