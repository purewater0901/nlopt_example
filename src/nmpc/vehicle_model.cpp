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

    double l = lon_velocity * dt + 0.5*dt*dt*lon_acc;
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

void VehicleModel::DynamicEquationConstraint(unsigned m,
                                             double* result,
                                             unsigned n,
                                             const double* x,
                                             double* grad,
                                             void* f_data)
{
    VehicleDynamicsData* data= reinterpret_cast<VehicleDynamicsData*>(f_data);
    int horizon = data->horizon_;
    int dim_x   = data->dim_x_;
    int dim_u   = data->dim_u_;
    int dim_xu_s = dim_x*horizon + dim_u*(horizon-1);
    double dt   = data->dt_;

    //assert(m == dim_x*(horizon-1));
    //assert(n == dim_xu_s);

    for(int i=0; i<horizon-1; ++i)
    {
        std::vector<double> x_next(dim_x, 0.0);
        double lon_velocity = x[i*dim_x+2];
        double yaw_curr     = x[i*dim_x+3];
        double lon_acc      = x[dim_x*horizon+i*dim_u];
        double curvature    = x[dim_x*horizon+i*dim_u+1];
        double l            = lon_velocity*dt + 0.5*dt*dt*lon_acc;
        double yaw_next     = yaw_curr + curvature*l;

        if(std::fabs(curvature)>1e-6)
        {
            x_next[0] = x[i*dim_x]   + (std::sin(yaw_next) - std::sin(yaw_curr))/curvature;
            x_next[1] = x[i*dim_x+1] + (std::cos(yaw_curr) - std::cos(yaw_next))/curvature;
        }
        else
        {
            x_next[0] = x[i*dim_x]   + l * std::cos(yaw_curr);
            x_next[1] = x[i*dim_x+1] + l * std::sin(yaw_curr);
        }
        x_next[2] = lon_velocity + lon_acc * dt;
        x_next[3] = yaw_next;

        result[i*dim_x]   = x[(i+1)*dim_x]   - x_next[0];
        result[i*dim_x+1] = x[(i+1)*dim_x+1] - x_next[1];
        result[i*dim_x+2] = x[(i+1)*dim_x+2] - x_next[2];
        result[i*dim_x+3] = x[(i+1)*dim_x+3] - x_next[3];
        /*
        result[i*3]   = x[(i+1)*dim_x]   - x_next[0];
        result[i*3+1] = x[(i+1)*dim_x+1] - x_next[1];
        result[i*3+2] = x[(i+1)*dim_x+2] - x_next[2];
         */

        if(grad)
        {
            int id_x   = (i*dim_x)  *dim_xu_s;
            int id_y   = (i*dim_x+1)*dim_xu_s;
            int id_v   = (i*dim_x+2)*dim_xu_s;
            int id_yaw = (i*dim_x+3)*dim_xu_s;
            /*
            int id_x   = (i*3)  *dim_xu_s;
            int id_y   = (i*3+1)*dim_xu_s;
            int id_v   = (i*3+2)*dim_xu_s;
             */

            if(std::fabs(curvature)>1e-6)
            {
                //gradient for x
                grad[id_x+i*dim_x]       = -1.0; // dc/d(x_curr)
                grad[id_x+i*dim_x+2]     = -dt*std::cos(yaw_next); // dc/d(v_curr)
                grad[id_x+i*dim_x+3]     =  (1.0/curvature)*std::cos(yaw_curr); // dc/d(yaw_curr)
                grad[id_x+(i+1)*dim_x]   =  1.0; // dc/d(x_next)
                grad[id_x+(i+1)*dim_x+3] = -(1.0/curvature)*std::cos(yaw_next); // dc/d(yaw_next)
                grad[id_x+horizon*dim_x+i*dim_u]   = -0.5*dt*dt*std::cos(yaw_next); // dc/d(u_a)
                grad[id_x+horizon*dim_x+i*dim_u+1] = (-curvature*l*std::cos(yaw_next)+std::sin(yaw_next)-std::sin(yaw_curr))/(curvature*curvature); // dc/d(u_k)

                //gradient for y
                grad[id_y+i*dim_x+1]     = -1.0; // dc/d(y_curr)
                grad[id_y+i*dim_x+2]     = -dt*std::sin(yaw_next); // dc/d(v_curr)
                grad[id_y+i*dim_x+3]     = (1.0/curvature)*std::sin(yaw_curr); // dc/d(y_curr)
                grad[id_y+(i+1)*dim_x+1] =  1.0; // dc/d(y_next)
                grad[id_y+(i+1)*dim_x+3] = -(1.0/curvature)*std::sin(yaw_next); // dc/d(y_next)
                grad[id_y+horizon*dim_x+i*dim_u]   = -0.5*dt*dt*std::sin(yaw_next); // dc/d(u_a)
                grad[id_y+horizon*dim_x+i*dim_u+1] = (-curvature*l*std::sin(yaw_next)+ std::cos(yaw_curr) - std::cos(yaw_next))/(curvature*curvature); // dc/d(u_k)
            }
            else
            {
                //gradient for x
                grad[id_x+i*dim_x]       = -1.0; // dc/d(x_curr)
                grad[id_x+i*dim_x+2]     = -dt*std::cos(yaw_curr); // dc/d(x_curr)
                grad[id_x+i*dim_x+3]     =   l*std::sin(yaw_curr); // dc/d(x_curr)
                grad[id_x+(i+1)*dim_x]   =  1.0; // dc/d(x_next)
                grad[id_x+horizon*dim_x+i*dim_u] = -0.5*dt*dt*std::cos(yaw_curr); // dc/d(x_next)

                //gradient for y
                grad[id_y+i*dim_x+1]     = -1.0; // dc/d(y_curr)
                grad[id_y+i*dim_x+2]     = -dt*std::sin(yaw_curr); // dc/d(v_curr)
                grad[id_y+i*dim_x+3]     =  -l*std::cos(yaw_curr); // dc/d(yaw_curr)
                grad[id_y+(i+1)*dim_x+1] =  1.0; // dc/d(y_next)
                grad[id_y+horizon*dim_x+i*dim_u] = -0.5*dt*dt*std::sin(yaw_curr); // dc/d(u_a)
            }

            //gradient for v
            grad[id_v+i*dim_x+2]             = -1.0;
            grad[id_v+(i+1)*dim_x+2]         =  1.0;
            grad[id_v+horizon*dim_x+i*dim_u] =  -dt;

            //gradient for yaw
            grad[id_yaw+i*dim_x+2]               = -curvature*dt;
            grad[id_yaw+i*dim_x+3]               = -1.0;
            grad[id_yaw+(i+1)*dim_x+3]           =  1.0;
            grad[id_yaw+horizon*dim_x+i*dim_u]   = -0.5*dt*dt*curvature;
            grad[id_yaw+horizon*dim_x+i*dim_u+1] = -l;
        }
    }
}









