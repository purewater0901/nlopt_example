#ifndef PRC_NLOPT_VEHICLE_MODEL_H
#define PRC_NLOPT_VEHICLE_MODEL_H

#include <iostream>
#include <memory>
#include <Eigen/Eigen>
#include <vector>
#include "utils.h"

class Vehicle
{
public:
    Vehicle(const int dim_x, const int dim_u);



    int getDimX(){return dim_x_;}
    int getDimU(){return dim_u_;}

    int dim_x_;
    int dim_u_;
};


#endif //PRC_NLOPT_VEHICLE_MODEL_H
