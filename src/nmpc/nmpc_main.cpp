#include <iostream>
#include "nmpc/nmpc_controller.h"
#include "nmpc/vehicle_model.h"
#include "nmpc/reference_trajectory_generator.h"

int main(int argc, char** argv)
{
    int dim_x = 4;
    int dim_u = 2;
    int horizon = 30;
    double dt = 0.1;
    double x_tolerance = 1e-7;
    double equality_tolerance = 1e-3;
    double inequality_tolerance = 1e-4;

    RefGenerator ref_generator;
    NMPCController controller(x_tolerance, equality_tolerance, inequality_tolerance);
    std::shared_ptr<VehicleModel> vehicle_ptr = std::make_shared<VehicleModel>(dim_x, dim_u);

    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(dim_x);
    x0(1) = 10;
    x0(2) = 2.0;
    x0(3) = M_PI/3;
    VecOfVecXd x_ref, u_ref;
    ref_generator.generate(x0, vehicle_ptr, dim_x, dim_u, horizon, dt, x_ref, u_ref);

    VecOfVecXd u_optimal;
    controller.calculateOptimalCommand(x_ref, u_ref, vehicle_ptr, dt, u_optimal);


    return 0;
}