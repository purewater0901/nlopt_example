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
    int dim_xu = dim_x*horizon+dim_u*(horizon-1);
    double wheel_length = 4.0;
    double steer_lim = M_PI * (45.0/180.0);
    double curvature_lim = std::tan(steer_lim)/wheel_length;
    double a_lim = 1.0;

    nlopt::opt opt(nlopt::LD_SLSQP, dim_xu);

    //1. create input constraint
    std::vector<double> lb(dim_xu);
    std::vector<double> ub(dim_xu);
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
    for(int i=dim_x*horizon; i<dim_xu; i+=dim_u)
    {
        lb[i] = -curvature_lim;
        ub[i] =  curvature_lim;
        lb[i+1] = -a_lim;
        ub[i+1] =  a_lim;
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
    Q(2,2) = 1.0;
    Q(3,3) = 10.0;
    R(0,0) = 0.1;
    R(1,1) = 0.1;
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
    VehicleDynamicsData vehicle_data;
    vehicle_data.horizon_ = horizon;
    vehicle_data.dt_ = dt;
    vehicle_data.dim_x_ = dim_x;
    vehicle_data.dim_u_ = dim_u;
    opt.add_equality_constraint(VehicleModel::test_constraint, &vehicle_data, 1e-8);

    //3.set x tolerance
    opt.set_xtol_rel(x_tolerance_);

    //4. set initial value
    VecOfVecXd  u_ini = std::vector<Eigen::VectorXd>(horizon-1, Eigen::VectorXd::Zero(dim_u));
    VecOfVecXd  x_ini = std::vector<Eigen::VectorXd>(horizon, Eigen::VectorXd::Zero(dim_x));
    x_ini.front() = x_ref.front();
    for(int i=0; i<horizon-1; ++i)
        x_ini[i+1] = vehicle_ptr->calcNextState(x_ini[i], u_ini[i], dt);
    std::vector<double> x(dim_xu);
    for(int i=0; i<horizon; i++)
        for(int j=0; j<dim_x; ++j)
            x[i*dim_x+j] = x_ini[i](j);
    for(int i=0; i<horizon-1; i++)
        for(int j=0; j<dim_u; ++j)
            x[horizon*dim_x+i*dim_u+j] = u_ini[i](j);

    //5. solve the problem
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
        std::cout << "x size: " << x.size() << std::endl;
        for(int i=0; i<horizon; i++)
        {
            std::cout << "x:     " << x[i*dim_x] << " " << x_ref[i](0) << std::endl;
            std::cout << "y:     " << x[i*dim_x+1] - x_ref[i](1) << std::endl;
            std::cout << "v:     " << x[i*dim_x+2] << " " <<  x_ref[i](2) << std::endl;
            std::cout << "yaw:   " << x[i*dim_x+3] - x_ref[i](3) << std::endl;
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

double NMPCController::objective_function(const std::vector<double>& x_vec,
                                          std::vector<double> &grad,
                                          void *my_func_data)
{
    ObjectiveData* data= reinterpret_cast<ObjectiveData*>(my_func_data);
    int horizon = data->horizon_;
    int dim_x = data->dim_x_;
    int dim_u = data->dim_u_;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(dim_x*horizon, dim_x*horizon);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim_u*(horizon-1), dim_u*(horizon-1));
    for(int i=0; i<horizon; ++i)
        Q.block(i*dim_x, i*dim_x, dim_x, dim_x) = data->weight_.Q_;
    for(int i=0; i<horizon-1; ++i)
        R.block(i*dim_u, i*dim_u, dim_u, dim_u) = data->weight_.R_;

    assert(x_vec.size() == dim_x * horizon +dim_u*(horizon-1));

    Eigen::VectorXd x_ref = data->ref_.x_ref_;
    Eigen::VectorXd u_ref = data->ref_.u_ref_;
    Eigen::VectorXd dx = Eigen::VectorXd::Zero(Q.rows());
    Eigen::VectorXd du = Eigen::VectorXd::Zero(R.rows());
    for(int i=0; i<dx.size(); ++i)
        dx(i) = x_vec[i] - x_ref(i);
    for(int i=0; i<du.size(); ++i)
        du(i) = x_vec[i+dim_x*horizon] - u_ref(i);

    if(!grad.empty())
    {
        assert(grad.size() == dim_x*horizon+dim_u*(horizon-1));
        Eigen::VectorXd x_dot = Q*dx;
        Eigen::VectorXd u_dot = R*du;

        for(int i=0; i<x_dot.size(); ++i)
            grad[i] = x_dot(i);
        for(int i=0; i<u_dot.size(); ++i)
            grad[i+x_dot.size()] = u_dot(i);
    }

    return 0.5*dx.transpose().dot(Q*dx) + 0.5*du.transpose().dot(R*du);
}