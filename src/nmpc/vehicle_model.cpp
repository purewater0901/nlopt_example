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

double VehicleModel::x_constraint(const std::vector<double> &x,
                                  std::vector<double> &grad,
                                  void *vehicle_data)
{
    VehicleDynamicsData* data= reinterpret_cast<VehicleDynamicsData*>(vehicle_data);
    int dim_x = data->dim_x_;
    int state_id = data->state_id_;
    int input_id = data->input_id_;
    double dt = data->dt_;

    double x_k     = x[state_id];
    double x_k_1   = x[state_id+dim_x];
    double v_k     = x[state_id+2];
    double yaw_k   = x[state_id+3];
    double yaw_k_1 = x[state_id+dim_x+3];
    double u_a = x[input_id];
    double u_k = x[input_id+1];
    double l = v_k * dt + 0.5*dt*dt*u_a;

    double dx;
    if(std::fabs(u_k)>1e-6)
        dx = x_k_1 - (x_k + (std::sin(yaw_k_1) - std::sin(yaw_k))/u_k);
    else
        dx = x_k_1 - (x_k + l * std::cos(yaw_k));

    if(!grad.empty())
    {
        if(std::fabs(u_k)>1e-6)
        {
            grad[state_id]         += -1.0;
            grad[state_id+3]       +=  (1.0/u_k)*std::cos(yaw_k);
            grad[state_id+dim_x]   +=  1.0;
            grad[state_id+dim_x+3] += -(1.0/u_k)*std::cos(yaw_k_1);
            grad[input_id+1]       += (std::sin(yaw_k_1) - std::sin(yaw_k))/(u_k * u_k);
        }
        else
        {
            grad[state_id]        += -1.0;
            grad[state_id+2]      += -dt * std::cos(yaw_k);
            grad[state_id+3]      +=  l * std::sin(yaw_k);
            grad[state_id+dim_x]  +=  1.0;
            grad[input_id]        += -0.5*dt*dt*std::cos(yaw_k);
        }
    }

    return dx;
}

double VehicleModel::y_constraint(const std::vector<double> &x,
                                  std::vector<double> &grad,
                                  void *vehicle_data)
{
    VehicleDynamicsData* data= reinterpret_cast<VehicleDynamicsData*>(vehicle_data);
    int dim_x = data->dim_x_;
    int state_id = data->state_id_;
    int input_id = data->input_id_;
    double dt = data->dt_;

    double y_k     = x[state_id+1];
    double y_k_1   = x[state_id+dim_x+1];
    double v_k     = x[state_id+2];
    double yaw_k   = x[state_id+3];
    double yaw_k_1 = x[state_id+dim_x+3];
    double u_a = x[input_id];
    double u_k = x[input_id+1];
    double l = v_k * dt + 1.0/2.0*dt*dt*u_a;

    double dy;
    if(std::fabs(u_k)>1e-6)
        dy = y_k_1 - (y_k + (std::cos(yaw_k) - std::cos(yaw_k_1))/u_k);
    else
        dy = y_k_1 - (y_k + l * std::sin(yaw_k));

    if(!grad.empty())
    {
        if(std::fabs(u_k)>1e-6)
        {
            grad[state_id+1]        += -1.0;
            grad[state_id+3]        += (1.0/u_k)*std::sin(yaw_k);
            grad[state_id+dim_x+1]  +=  1.0;
            grad[state_id+dim_x+3]  +=  -(1.0/u_k)*std::sin(yaw_k_1);
            grad[input_id+1]        += (std::cos(yaw_k) - std::cos(yaw_k_1))/(u_k * u_k);
        }
        else
        {
            grad[state_id+1]        += -1.0;
            grad[state_id+2]        += -dt * std::sin(yaw_k);
            grad[state_id+3]        +=  -l * std::cos(yaw_k);
            grad[state_id+dim_x+1]  +=  1.0;
            grad[input_id]          += -0.5*dt*dt*std::sin(yaw_k);
        }
    }

    return dy;
}

double VehicleModel::v_constraint(const std::vector<double> &x,
                                  std::vector<double> &grad,
                                  void *vehicle_data)
{
    VehicleDynamicsData* data= reinterpret_cast<VehicleDynamicsData*>(vehicle_data);
    int horizon = data->horizon_;
    int dim_x = data->dim_x_;
    int dim_u = data->dim_u_;
    double dt = data->dt_;

    double a = 0.0;
    for(int i=0; i<horizon; ++i)
        a += x[horizon*dim_x + i*dim_u];

    if(!grad.empty())
    {
        grad[2] += -1.0;
        grad[(horizon-1)*dim_x+2] += 1.0;
        for(int i=0; i<horizon-1; ++i)
            grad[(horizon*dim_x + i*dim_u)] += -dt;
    }

    return x[(horizon-1)*dim_x+2] - x[2] - a*dt;
}

double VehicleModel::yaw_constraint(const std::vector<double> &x,
                                  std::vector<double> &grad,
                                  void *vehicle_data)
{
    VehicleDynamicsData* data= reinterpret_cast<VehicleDynamicsData*>(vehicle_data);
    int horizon = data->horizon_;
    int dim_x = data->dim_x_;
    int dim_u = data->dim_u_;
    double dt = data->dt_;

    if(!grad.empty())
    {
        grad[3] += -1.0;
        grad[(horizon-1)*dim_x+3] += 1.0;
    }

    double lk = 0.0;
    for(int i=0; i<horizon-1; ++i)
    {
        double u_a = x[horizon*dim_x + i*dim_u];
        double u_k = x[horizon*dim_x + i*dim_u + 1];
        double l =  x[i*dim_x+2]*dt + 1.0/2.0*dt*dt*u_a;
        lk += l*u_k;

        if(!grad.empty())
        {
            grad[i*dim_x + 2] += -u_k*dt;
            grad[horizon*dim_x + i*dim_u] += -0.5*dt*dt*u_k;
            grad[horizon*dim_x + i*dim_u + 1] += -l;
        }
    }

    double result = x[(horizon-1)*dim_x+3] - x[3] - lk;
    return NMPCUtils::normalizeRadian(result);
}

double VehicleModel::test_constraint(const std::vector<double>& x, std::vector<double>& grad, void* vehicle_data)
{
    VehicleDynamicsData* data= reinterpret_cast<VehicleDynamicsData*>(vehicle_data);

    for(int i=0; i<grad.size(); ++i)
        grad[i] = 0.0;

    double dh = 0.0;
    /*
    for(int i=0; i<data->horizon_; ++i)
    {
        VehicleDynamicsData new_data;
        new_data.horizon_ = data->horizon_;
        new_data.dt_ = data->dt_;
        new_data.dim_x_ = data->dim_x_;
        new_data.dim_u_ = data->dim_u_;
        new_data.state_id_ = i*data->dim_x_;
        new_data.input_id_ = data->horizon_*data->dim_x_+ i*data->dim_u_;

        //dh += x_constraint(x, grad, &new_data);
        //dh += y_constraint(x, grad, &new_data);
    }
     */
    VehicleDynamicsData new_data;
    new_data.horizon_ = data->horizon_;
    new_data.dt_ = data->dt_;
    new_data.dim_x_ = data->dim_x_;
    new_data.dim_u_ = data->dim_u_;
    dh += v_constraint(x, grad, &new_data);
    dh += yaw_constraint(x, grad, &new_data);

    return dh;
}










