#include "nmpc/vehicle_model.h"

VehicleModel::VehicleModel(const int dim_x, const int dim_u) : dim_x_(dim_x), dim_u_(dim_u)
{

}

Eigen::VectorXd VehicleModel::calcNextState(const Eigen::VectorXd &x_curr,
                                            const Eigen::VectorXd &u,
                                            const double dt)
{
    assert(x_curr.size() == dim_x_);
    assert(u.size() == dim_u_);
    Eigen::VectorXd x_next = Eigen::VectorXd::Zero(dim_x_);

    double lon_velocity = x_curr(2);
    double yaw_curr = x_curr(3);
    double lon_acc = u(0);
    double curvature = u(1);

    double l = lon_velocity * dt + 1.0/2.0*dt*dt*lon_acc;
    double yaw_next = NMPCUtils::normalizeRadian(yaw_curr + curvature*l);
    if(std::fabs(curvature)>1e-6)
    {
        x_next(0) = x_curr(0) + (std::sin(yaw_next) - std::sin(yaw_curr))/curvature;
        x_next(1) = x_curr(1) + (std::cos(yaw_curr) - std::cos(yaw_next))/curvature;
    }
    else
    {
        x_next(0) = x_curr(0) + l * std::cos(yaw_curr);
        x_next(1) = x_curr(1) + l * std::sin(yaw_curr);
    }
    x_next(2) = lon_velocity + lon_acc * dt;
    x_next(3) = yaw_next;

    return x_next;

}
