#ifndef PRC_NLOPT_NMPC_CONTROLLER_H
#define PRC_NLOPT_NMPC_CONTROLLER_H

#include <iostream>
#include <vector>
#include <nlopt.hpp>
#include <cassert>
#include <cmath>
#include <memory>
#include <iomanip>
#include <Eigen/Eigen>
#include <chrono>
#include "nmpc_utils.h"
#include "nmpc/vehicle_model.h"

typedef struct
{
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;
}Weight;

typedef struct
{
    Eigen::VectorXd x_ref_;
    Eigen::VectorXd u_ref_;
}Reference;

typedef struct
{
    Reference ref_;
    Weight weight_;
    int dim_x_;
    int dim_u_;
    int horizon_;
}ObjectiveData;

class NMPCController
{
public:
    NMPCController(const double x_tolerance,
                   const double equality_tolerance) : x_tolerance_(x_tolerance),
                                                      equality_tolerance_(equality_tolerance) {};

    bool calculateOptimalCommand(const VecOfVecXd& x_ref,
                                 const VecOfVecXd& u_ref,
                                 const std::shared_ptr<VehicleModel>& vehicle_ptr,
                                 const double dt,
                                 VecOfVecXd& optimal_command);

    static double objective_function(const std::vector<double>& x_vec,
                                     std::vector<double>& grad,
                                     void* my_func_data);
private:
    double x_tolerance_;
    double equality_tolerance_;
};

#endif //PRC_NLOPT_NMPC_CONTROLLER_H
