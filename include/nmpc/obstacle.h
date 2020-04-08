#ifndef PRC_NLOPT_OBSTACLE_H
#define PRC_NLOPT_OBSTACLE_H

class Obstacle
{
public:
    Obstacle() = default;
    Obstacle(const double x, const double y, const double radius);

    double x_;
    double y_;
    double radius_;
};

#endif //PRC_NLOPT_OBSTACLE_H
