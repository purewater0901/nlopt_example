#ifndef PRC_NLOPT_VEHICLE_MODEL_H
#define PRC_NLOPT_VEHICLE_MODEL_H

#include <iostream>
#include <memory>
#include <Eigen/Eigen>
#include <vector>
#include "nmpc/nmpc_utils.h"

typedef struct
{
    int dim_x_;
    int dim_u_;
    int horizon_;
    double dt_;
}VehicleDynamicsData;

class VehicleModel
{
public:
    VehicleModel(const int dim_x, const int dim_u);

    Eigen::VectorXd calcNextState(const Eigen::VectorXd& x_curr, const Eigen::VectorXd& u, const double dt);

    double vehicle_constraint(const std::vector<double>& x, std::vector<double>& grad, void* data);

    int getDimX(){return dim_x_;}
    int getDimU(){return dim_u_;}

    int dim_x_;
    int dim_u_;
};


#endif //PRC_NLOPT_VEHICLE_MODEL_H
