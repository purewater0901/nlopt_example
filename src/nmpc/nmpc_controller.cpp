#include <nmpc/vehicle_model.h>
#include "nmpc/nmpc_controller.h"

bool NMPCController::calculateOptimalCommand(const VecOfVecXd& x_ref,
                                             const VecOfVecXd& u_ref,
                                             const std::shared_ptr<VehicleModel>& vehicle_ptr,
                                             const double dt,
                                             VecOfVecXd& optimal_command)
{
    int dim_x = x_ref.front().size();
    int dim_u = u_ref.front().size();
    int horizon = x_ref.size();
    int dim_xu_s = dim_x*horizon+dim_u*(horizon-1);
    double wheel_length = 4.0;
    double steer_lim = M_PI * (45.0/180.0);
    double curvature_lim = std::tan(steer_lim)/wheel_length;
    double a_lim = 1.0;

    //nlopt::opt opt(nlopt::LN_COBYLA, dim_xu_s);
    nlopt::opt opt(nlopt::LD_SLSQP, dim_xu_s);

    //1. create input constraint
    std::vector<double> lb(dim_xu_s);
    std::vector<double> ub(dim_xu_s);
    for(int i=0; i<horizon; ++i)
    {
        lb[i*dim_x] = -HUGE_VAL;
        ub[i*dim_x] =  HUGE_VAL;
        lb[i*dim_x+1] = -HUGE_VAL;
        ub[i*dim_x+1] =  HUGE_VAL;
        lb[i*dim_x+2] =  -HUGE_VAL;
        ub[i*dim_x+2] =  HUGE_VAL;
        lb[i*dim_x+3] = -HUGE_VAL;
        ub[i*dim_x+3] =  HUGE_VAL;
    }
    for(int i=dim_x*horizon; i<dim_xu_s; i+=dim_u)
    {
        lb[i] = -a_lim;
        ub[i] =  a_lim;
        lb[i+1] = -curvature_lim;
        ub[i+1] =  curvature_lim;
    }
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);

    //2. set objective function
    Eigen::VectorXd x_ref_eigen = Eigen::VectorXd::Zero(dim_x*x_ref.size());
    Eigen::VectorXd u_ref_eigen = Eigen::VectorXd::Zero(dim_u*u_ref.size());
    for(int i=0; i<x_ref.size(); ++i)
        for(int j=0; j<dim_x; ++j)
            x_ref_eigen(i*dim_x + j) = x_ref[i](j);
    for(int i=0; i<u_ref.size(); ++i)
        for(int j=0; j<dim_u; ++j)
            u_ref_eigen(i*dim_u + j) = u_ref[i](j);
    Reference ref;
    ref.x_ref_ = x_ref_eigen;
    ref.u_ref_ = u_ref_eigen;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(dim_x, dim_x);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_u, dim_u);
    Q(0,0) = 10.0;
    Q(1,1) = 10.0;
    Q(2,2) = 10.0;
    Q(3,3) = 10.0;
    R(0,0) = 1.0;
    R(1,1) = 10.0;
    Weight W;
    W.Q_ = Q;
    W.R_ = R;

    ObjectiveData data;
    data.ref_ = ref;
    data.weight_ = W;
    data.dim_x_ = dim_x;
    data.dim_u_ = dim_u;
    data.horizon_ = horizon;
    opt.set_min_objective(this->objective_function, &data);

    //3. add constraints
    //equality constraints
    VehicleDynamicsData vehicle_data;
    vehicle_data.horizon_ = horizon;
    vehicle_data.dt_ = dt;
    vehicle_data.dim_x_ = dim_x;
    vehicle_data.dim_u_ = dim_u;
    std::vector<double> tol_vehicle_equality(dim_x*(horizon-1), equality_tolerance_);
    opt.add_equality_mconstraint(VehicleModel::DynamicEquationConstraint, &vehicle_data, tol_vehicle_equality);

    //inequality constraints(obstacle avoidance)
    Obstacle obs(10, 0, 1);
    ObsAvoidData obs_constrain_data;
    obs_constrain_data.dim_x_   = dim_x;
    obs_constrain_data.dim_u_   = dim_u;
    obs_constrain_data.horizon_ = horizon;
    obs_constrain_data.dt_ = dt;
    std::vector<double> tol_obstacle_avoidance(horizon, inequality_tolerance_);
    opt.add_inequality_mconstraint(this->obstacle_constraint, &obs_constrain_data, tol_obstacle_avoidance);


    //4.set x tolerance
    opt.set_xtol_rel(x_tolerance_);

    //5. set initial value
    VecOfVecXd  u_ini = std::vector<Eigen::VectorXd>(horizon-1, Eigen::VectorXd::Zero(dim_u));
    VecOfVecXd  x_ini = std::vector<Eigen::VectorXd>(horizon, Eigen::VectorXd::Zero(dim_x));
    x_ini.front() = x_ref.front();
    for(int i=0; i<horizon-1; ++i)
        x_ini[i+1] = vehicle_ptr->calcNextState(x_ini[i], u_ini[i], dt);
    std::vector<double> x(dim_xu_s);
    for(int i=0; i<horizon; i++)
        for(int j=0; j<dim_x; ++j)
            x[i*dim_x+j] = x_ini[i](j);
    for(int i=0; i<horizon-1; i++)
        for(int j=0; j<dim_u; ++j)
            x[horizon*dim_x+i*dim_u+j] = u_ini[i](j);

    //6. solve the problem
    double min_f;
    try
    {
        //solve the problem
        std::chrono::system_clock::time_point start, end;
        start = std::chrono::system_clock::now();

        nlopt::result result = opt.optimize(x, min_f);

        end = std::chrono::system_clock::now();
        double time = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0);
        std::cout << "Calculation time: " << time << "[ms]" << std::endl;

        std::cout << "nlopt success" << std::endl;
        std::cout << "Minimum Value: " << min_f << std::endl;
        for(int i=0; i<horizon; i++)
        {
            std::cout << "x:     " << x[i*dim_x]   << " " << x_ref[i](0) << std::endl;
            std::cout << "y:     " << x[i*dim_x+1] << " " << x_ref[i](1) << std::endl;
            std::cout << "v:     " << x[i*dim_x+2] << " " << x_ref[i](2) << std::endl;
            std::cout << "yaw:   " << x[i*dim_x+3] << " " << x_ref[i](3) << std::endl;
        }

        for(int i=0; i<horizon-1; ++i)
        {
            std::cout << "u_a: " << x[horizon*dim_x+i*dim_u] << std::endl;
            std::cout << "u_k: " << x[horizon*dim_x+i*dim_u+1] << std::endl;
        }

        optimal_command.resize(horizon-1);
        for(int i=0; i<horizon-1; i++)
        {
            Eigen::VectorXd u_tmp = Eigen::VectorXd::Zero(dim_u);
            u_tmp(0) = x[horizon*dim_x + dim_u*i];
            u_tmp(1) = x[horizon*dim_x + dim_u*i+1];
            optimal_command[i] = u_tmp;
        }

        return true;
    }
    catch(std::exception & e)
    {
        std::cout << "nlopt failed: " << e.what() << std::endl;
        return false;
    }
}

double NMPCController::objective_function(unsigned n,
                                          const double* x_vec,
                                          double *grad,
                                          void* objective_data)
{
    ObjectiveData* data= reinterpret_cast<ObjectiveData*>(objective_data);
    int horizon = data->horizon_;
    int dim_x = data->dim_x_;
    int dim_u = data->dim_u_;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(dim_x*horizon, dim_x*horizon);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_u*(horizon-1), dim_u*(horizon-1));
    for(int i=0; i<horizon; ++i)
        Q.block(i*dim_x, i*dim_x, dim_x, dim_x) = data->weight_.Q_;
    for(int i=0; i<horizon-1; ++i)
        R.block(i*dim_u, i*dim_u, dim_u, dim_u) = data->weight_.R_;

    assert(n == dim_x * horizon +dim_u*(horizon-1));

    Eigen::VectorXd x_ref = data->ref_.x_ref_;
    Eigen::VectorXd u_ref = data->ref_.u_ref_;
    Eigen::VectorXd dx = Eigen::VectorXd::Zero(Q.rows());
    Eigen::VectorXd du = Eigen::VectorXd::Zero(R.rows());
    for(int i=0; i<dx.size(); ++i)
        dx(i) = x_vec[i] - x_ref(i);
    for(int i=0; i<du.size(); ++i)
        du(i) = x_vec[i+dim_x*horizon] - u_ref(i);

    Eigen::VectorXd qdx = Q*dx;
    Eigen::VectorXd rdu = R*du;

    if(grad)
    {
        assert(n == dim_x*horizon+dim_u*(horizon-1));
        assert(n == qdx.size()+rdu.size());

        for(int i=0; i<qdx.size(); ++i)
            grad[i] = qdx(i);
        for(int i=0; i<rdu.size(); ++i)
            grad[i+horizon*dim_x] = rdu(i);
    }

    return 0.5*dx.transpose().dot(qdx) + 0.5*du.transpose().dot(rdu);
}

void NMPCController::obstacle_constraint(unsigned m,
                                         double* result,
                                         unsigned n,
                                         const double* x_vec,
                                         double* grad,
                                         void* obstacle_data)
{
    ObsAvoidData* data= reinterpret_cast<ObsAvoidData*>(obstacle_data);

    double obs_x = data->obstacle_.x_;
    double obs_y = data->obstacle_.y_;
    double obs_r = data->obstacle_.radius_;
    int dim_x   = data->dim_x_;
    int dim_u   = data->dim_u_;
    int horizon = data->horizon_;
    double dt   = data->dt_;
    int dim_xu_s = dim_x*horizon+dim_u*(horizon-1);

    assert(m==horizon);

    for(int i=0; i<horizon; ++i)
    {
        double ego_r = 3.0;
        double ego_x = x_vec[i*dim_x];
        double ego_y = x_vec[i*dim_x+1];
        double dx = ego_x-obs_x;
        double dy = ego_y-obs_y;

        if(grad)
        {
            int id = i*dim_xu_s;
            grad[id+i*dim_x]   = -2*dx;
            grad[id+i*dim_x+1] = -2*dy;

            if(i>0)
            {
                double u_a = x_vec[horizon*dim_x+(i-1)*dim_u];
                double u_k = x_vec[horizon*dim_x+(i-1)*dim_u+1];
                double yaw_curr = x_vec[(i-1)*dim_x+3];
                double yaw_next = x_vec[i*dim_x+3];
                double lon_velocity = x_vec[(i-1)*dim_x+2];
                double l = lon_velocity*dt + 0.5*dt*dt*u_a;

                if(u_k>1e-6)
                {
                    grad[id+(i-1)*dim_x]   = -2*dx; // dc/d(x_curr)
                    grad[id+(i-1)*dim_x+1] = -2*dy; // dc/d(x_curr)
                    grad[id+(i-1)*dim_x+2] = -2*dx*(-dt*std::cos(yaw_next)) - 2*dy*(-dt*std::sin(yaw_next)); // dc/d(v_curr)
                    grad[id+(i-1)*dim_x+3] = -2*dx*(1.0/u_k)*std::cos(yaw_curr) - 2*dy*(1.0/u_k)*std::sin(yaw_curr); // dc/d(yaw_curr)
                    grad[id+i*dim_x+3]     = -2*dx*(-(1.0/u_k)*std::cos(yaw_next)) -2*dy*(-(1.0/u_k)*std::sin(yaw_next)); // dc/d(yaw_next)
                    grad[id+horizon*dim_x+(i-1)*dim_u]   = -2*dx*(-0.5*dt*dt*std::cos(yaw_next)) - 2*dy*(-0.5*dt*dt*std::sin(yaw_next)); // dc/d(u_a)
                    grad[id+horizon*dim_x+(i-1)*dim_u+1] = -2*dx*(-u_k*l*std::cos(yaw_next)+std::sin(yaw_next)-std::sin(yaw_curr))/(u_k*u_k)
                                                           -2*dy*(-u_k*l*std::sin(yaw_next)+std::cos(yaw_curr)-std::cos(yaw_next))/(u_k*u_k); // dc/d(u_k)
                }
                else
                {
                    grad[id+(i-1)*dim_x]   = -2*dx; // dc/d(x_curr)
                    grad[id+(i-1)*dim_x+1] = -2*dy; // dc/d(y_curr)
                    grad[id+(i-1)*dim_x+2] = -2*dx*(-dt*std::cos(yaw_curr)) - 2*dy*(-dt*std::sin(yaw_curr)); // dc/d(v_curr)
                    grad[id+(i-1)*dim_x+3] = -2*dx*(l*std::sin(yaw_curr))   - 2*dy*(-l*std::cos(yaw_curr)); // dc/d(yaw_curr)
                    grad[id+horizon*dim_x+(i-1)*dim_u] = -2*dx*(-0.5*dt*dt*std::cos(yaw_curr))-2*dy*(-0.5*dt*dt*std::sin(yaw_curr)); // dc/d(x_next)
                }
            }
        }

        result[i] = (ego_r+obs_r)*(ego_r+obs_r) - dx*dx - dy*dy;
    }
}
