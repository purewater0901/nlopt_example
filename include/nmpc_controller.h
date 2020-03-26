#ifndef PRC_NLOPT_NMPC_CONTROLLER_H
#define PRC_NLOPT_NMPC_CONTROLLER_H

#include <iostream>
#include <vector>
#include <nlopt.hpp>
#include <cassert>
#include <cmath>
#include <Eigen/Eigen>
#include "utils.h"

typedef struct
{
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
}Weight;

class NMPCController
{
public:
    NMPCController();

    bool calculateOptimalCommand(const VecOfVecXd& reference_trajectory,
                                 Eigen::VectorXd& optimal_command);

    double objective_function(const Eigen::VectorXd& x,
                              std::vector<double>& grad,
                              void* my_func_data);

};

#endif //PRC_NLOPT_NMPC_CONTROLLER_H
