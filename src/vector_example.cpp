#include <iostream>
#include <vector>
#include <nlopt.hpp>
#include <cmath>
#include <iomanip>

double objective_function(unsigned n, const double* x, double *grad, void* objective_data)
{
    if(grad)
    {
        grad[0] = -4*(x[1]-x[0]*x[0])*x[0] - 2*(1-x[0]);
        grad[1] =  2*(x[1]-x[0]*x[0]);
    }

    return (x[1]-x[0]*x[0])*(x[1]-x[0]*x[0]) + (1-x[0])*(1-x[0]);
}

void vector_constraints(unsigned m, double* result, unsigned n, const double* x, double* grad, void* f_data)
{
    if(grad)
    {
        grad[0] = -x[1];
        grad[1] = -x[0];
        grad[2] = 0.5*x[0];
        grad[3] = -1.0;
    }
    result[0] = 4 - x[0]*x[1];
    result[1] = 2 + 0.25*x[0]*x[0] - x[1];

    return;
}

int main(int argc, char** argv)
{
    std::vector<double> tol(2, 1e-3);
    nlopt::opt opt(nlopt::LD_SLSQP, 2);

    opt.set_min_objective(objective_function, NULL);
    opt.add_inequality_mconstraint(vector_constraints, NULL, tol);

    opt.set_xtol_rel(1e-4);

    std::vector<double> x(2);
    x[0] = 2.0;
    x[1] = 4.0;

    double min_f;
    try
    {
        nlopt::result result = opt.optimize(x, min_f);
        std::cout << "find minmum at f(" << x[0] << "," << x[1] << ") = " << std::setprecision(10) << min_f << std::endl;
    }
    catch(std::exception& e)
    {
        std::cout << "nlopt failed: " << e.what() << std::endl;
    }

    return 0;
}
