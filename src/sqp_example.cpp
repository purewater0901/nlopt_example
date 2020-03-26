#include <iostream>
#include <vector>
#include <nlopt.hpp>
#include <cmath>
#include <iomanip>

typedef struct {
    double a, b;
} my_constraint_data;

double objective_function(const std::vector<double> &x, std::vector<double>& grad, void* my_func_data)
{
    if(!grad.empty())
    {
        grad[0] =  std::cos(x[0])*std::cos(x[1]) - std::sin(x[0])*std::sin(x[1]);
        grad[1] = -std::sin(x[0])*std::sin(x[1]) + std::cos(x[0])*std::cos(x[1]);
    }

    return std::sin(x[0])*std::cos(x[1]) + std::cos(x[0])*std::sin(x[1]);
}

double constraint1(const std::vector<double>& x, std::vector<double>& grad, void* data)
{
    my_constraint_data* d = reinterpret_cast<my_constraint_data*>(data);
    double a = d->a;
    double b = d->b;

    if(!grad.empty())
    {
        grad[0] = a;
        grad[1] = b;
    }
    return a*x[0] + b*x[1] - M_PI;
}

double constraint2(const std::vector<double>& x, std::vector<double>& grad, void* data)
{
    my_constraint_data* d = reinterpret_cast<my_constraint_data*>(data);
    double a = d->a;
    double b = d->b;

    if(!grad.empty())
    {
        grad[0] = a;
        grad[1] = b;
    }

    return a * x[0] + b*x[1];
}

double equality_constraint(const std::vector<double>& x, std::vector<double>& grad, void* data)
{
    if(!grad.empty())
    {
        grad[0] = 1.0;
        grad[1] = -3.0 * x[1] * x[1];
    }

    return x[0] - x[1]*x[1]*x[1];
}


int main(int argc, char** argv)
{
    nlopt::opt opt(nlopt::LD_SLSQP, 2); //algorithm and dimension

    opt.set_max_objective(objective_function, NULL);
    my_constraint_data data[2] = { {1, 1}, {-1, -1}};
    opt.add_inequality_constraint(constraint1, &data[0], 1e-8);
    opt.add_inequality_constraint(constraint2, &data[1], 1e-8);
    opt.add_equality_constraint(equality_constraint, NULL, 1e-8);

    opt.set_xtol_rel(1e-4);

    //initial guess
    std::vector<double> x(2);
    x[0] = 1.234;
    x[1] = 5.678;

    double max_f;
    try
    {
        nlopt::result result = opt.optimize(x, max_f);
        std::cout << "find minmum at f(" << x[0] << "," << x[1] << ") = " << std::setprecision(10) << max_f << std::endl;
    }
    catch(std::exception & e)
    {
        std::cout << "nlopt failed: " << e.what() << std::endl;
    }



    return 0;
}
