#ifndef BARNES_HUT_DEMO_ACCELERATION_H
#define BARNES_HUT_DEMO_ACCELERATION_H

#include <eigen3/Eigen/Dense>
#include "constants.h"

typedef Eigen::Vector2d vec2;

namespace Acceleration
{
    class Gravitational
    {
    public:
        vec2 operator()(const double RootMass, const vec2 &dist)
        {
            double norm = dist.norm();

            return -((GRAVITY_CONST * RootMass)
                   / (std::pow((norm * norm) + (EPSILON * EPSILON), 1.5))) * dist;
        }
    };
}

#endif //BARNES_HUT_DEMO_ACCELERATION_H
