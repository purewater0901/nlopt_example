#ifndef PRC_NLOPT_VEHICLE_MODEL_H
#define PRC_NLOPT_VEHICLE_MODEL_H

#include <iostream>
#include <memory>
#include <Eigen/Eigen>
#include <vector>
#include "nmpc/nmpc_utils.h"

typedef struct
{
    int horizon_;
    int dim_x_;
    int dim_u_;
    double dt_;
    int state_id_;
    int input_id_;
}VehicleDynamicsData;

class VehicleModel
{
public:
    VehicleModel(const int dim_x, const int dim_u);

    Eigen::VectorXd calcNextState(const Eigen::VectorXd& x_curr, const Eigen::VectorXd& u, const double dt);

    static double x_constraint(const std::vector<double>&x, std::vector<double>& grad, void* vehicle_data);
    static double y_constraint(const std::vector<double>&x, std::vector<double>& grad, void* vehicle_data);
    static double v_constraint(const std::vector<double>&x, std::vector<double>& grad, void* vehicle_data);
    static double yaw_constraint(const std::vector<double>&x, std::vector<double>& grad, void* vehicle_data);
    static double test_constraint(const std::vector<double>& x, std::vector<double>& grad, void* vehicle_data);

    int getDimX(){return dim_x_;}
    int getDimU(){return dim_u_;}

    int dim_x_;
    int dim_u_;
};


#endif //PRC_NLOPT_VEHICLE_MODEL_H
