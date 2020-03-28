#ifndef PRC_NLOPT_REFERENCE_TRAJECTORY_GENERATOR_H
#define PRC_NLOPT_REFERENCE_TRAJECTORY_GENERATOR_H

#include <iostream>
#include <vector>
#include <memory>
#include <Eigen/Eigen>
#include "nmpc/nmpc_utils.h"
#include "nmpc/vehicle_model.h"

class RefGenerator
{
public:
    RefGenerator() = default;

    void generate(const Eigen::VectorXd& x0,
                  const std::shared_ptr<VehicleModel>& vehicle_ptr,
                  const int horizon,
                  const int dim_x,
                  const int dim_u,
                  const double dt,
                  VecOfVecXd& x_ref,
                  VecOfVecXd& u_ref);

};

#endif //PRC_NLOPT_REFERENCE_TRAJECTORY_GENERATOR_H
