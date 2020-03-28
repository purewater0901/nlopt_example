#ifndef PRC_NLOPT_NMPC_UTILS_H
#define PRC_NLOPT_NMPC_UTILS_H

#include <iostream>
#include <vector>
#include <Eigen/Eigen>

typedef std::vector<Eigen::VectorXd> VecOfVecXd;
typedef std::vector<Eigen::MatrixXd> VecOfMatXd;

namespace NMPCUtils
{
    double normalizeRadian(const double _angle);
}

#endif //PRC_NLOPT_NMPC_UTILS_H
