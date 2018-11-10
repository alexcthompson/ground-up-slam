#ifndef UTILS
#define UTILS

#include <cmath>

double wrap_angle(double ang) {
    while (ang > M_PI) ang -= 2 * M_PI;
    while (ang <= -M_PI) ang += 2 * M_PI;
    return ang;
}

#endif // UTILS