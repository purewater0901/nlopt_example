#include "nmpc/nmpc_utils.h"

double NMPCUtils::normalizeRadian(const double _angle)
{
    double n_angle = std::fmod(_angle, 2 * M_PI);
    n_angle = n_angle > M_PI ? n_angle - 2 * M_PI : n_angle < -M_PI ? 2 * M_PI + n_angle : n_angle;

    return n_angle;
}
