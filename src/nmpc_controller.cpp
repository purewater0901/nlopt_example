#include "nmpc_controller.h"

int main(int argc, char** argv)
{

    return 0;
}

double NMPCController::objective_function(const Eigen::VectorXd& x,
                                          std::vector<double> &grad,
                                          void *my_func_data)
{
    Weight* weight = reinterpret_cast<Weight*>(my_func_data);
    Eigen::MatrixXd Q = weight->Q;
    Eigen::MatrixXd R = weight->R;

}