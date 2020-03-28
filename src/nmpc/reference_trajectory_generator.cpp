#include "nmpc/reference_trajectory_generator.h"

void RefGenerator::generate(const Eigen::VectorXd& x0,
                            const std::shared_ptr<VehicleModel>& vehicle_ptr,
                            const int dim_x,
                            const int dim_u,
                            const int horizon,
                            const double dt,
                            VecOfVecXd& x_ref,
                            VecOfVecXd& u_ref)
{
    x_ref = std::vector<Eigen::VectorXd>(horizon, Eigen::VectorXd::Zero(dim_x));
    u_ref = std::vector<Eigen::VectorXd>(horizon-1, Eigen::VectorXd::Zero(dim_u));

    x_ref.front() = x0;
    for(int i=0; i<u_ref.size(); ++i)
    {
        u_ref[i](0) = 0.1;
        u_ref[i](1) = 0.003;
    }

    for(int i=0; i<u_ref.size(); ++i)
        x_ref[i+1] = vehicle_ptr->calcNextState(x_ref[i], u_ref[i], dt);
}
