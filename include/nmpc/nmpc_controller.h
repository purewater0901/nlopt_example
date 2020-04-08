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
#include "nmpc/obstacle.h"

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

typedef struct
{
    Obstacle obstacle_;
    int dim_x_;
    int dim_u_;
    int horizon_;
    double dt_;
}ObsAvoidData;

class NMPCController
{
public:
    NMPCController(const double x_tolerance,
                   const double equality_tolerance,
                   const double inequality_tolerance) : x_tolerance_(x_tolerance),
                                                        equality_tolerance_(equality_tolerance),
                                                        inequality_tolerance_(inequality_tolerance){};

    bool calculateOptimalCommand(const VecOfVecXd& x_ref,
                                 const VecOfVecXd& u_ref,
                                 const std::shared_ptr<VehicleModel>& vehicle_ptr,
                                 const double dt,
                                 VecOfVecXd& optimal_command);

    static double objective_function(unsigned n,
                                     const double* x,
                                     double *grad,
                                     void* objective_data);

    static void obstacle_constraint(unsigned m,
                                    double* result,
                                    unsigned n,
                                    const double* x,
                                    double* grad,
                                    void* f_data);
private:
    double x_tolerance_;
    double equality_tolerance_;
    double inequality_tolerance_;
};

#endif //PRC_NLOPT_NMPC_CONTROLLER_H
