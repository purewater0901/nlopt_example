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

double VehicleModel::vehicle_constraint(const std::vector<double> &x, std::vector<double> &grad, void *vehicle_data)
{
    VehicleDynamicsData* data= reinterpret_cast<VehicleDynamicsData*>(vehicle_data);
    int dim_x = data->dim_x_;
    int dim_u = data->dim_u_;
    int horizon = data->horizon_;
    double dt = data->dt_;

    double ret = 0.0;
    double x_curr = x[0];
    double y_curr = x[1];
    double v_curr = x[2];
    double yaw_curr = x[3];

    if(!grad.empty())
    {

    }

    for(int i=0; i<horizon-1; ++i)
    {
        double lon_acc   = x[dim_x*horizon + dim_u*i];
        double curvature = x[dim_x*horizon + dim_u*i+1];
        double l = v_curr * dt + 1.0/2.0*dt*dt*lon_acc;

        double x_next;
        double y_next;
        double v_next = v_curr + lon_acc * dt;
        double yaw_next = NMPCUtils::normalizeRadian(yaw_curr + curvature*l);
        if(std::fabs(curvature)>1e-6)
        {
            x_next = x_curr + (std::sin(yaw_next) - std::sin(yaw_curr))/curvature;
            y_next = y_curr + (std::cos(yaw_curr) - std::cos(yaw_next))/curvature;
        }
        else
        {
            x_next = x_curr + l * std::cos(yaw_curr);
            y_next = y_curr + l * std::sin(yaw_curr);
        }

        double dx   = x[(i+1)*dim_x]   - x_next;
        double dy   = x[(i+1)*dim_x+1] - y_next;
        double dv   = x[(i+1)*dim_x+2] - v_next;
        double dyaw = x[(i+1)*dim_x+3] - yaw_next;
        ret += (dx+dy+dv+dyaw);

        x_curr = x_next;
        y_curr = y_next;
        v_curr = v_next;
        yaw_curr = yaw_next;
    }

    return ret;
}
