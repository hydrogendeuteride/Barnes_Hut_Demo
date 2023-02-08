#ifndef BARNES_HUT_DEMO_ACCELERATION_H
#define BARNES_HUT_DEMO_ACCELERATION_H

#include <eigen3/Eigen/Dense>

typedef Eigen::Vector2d vec2;

namespace Acceleration
{
    class Gravitational
    {
    public:
        vec2 operator()(const double LeafMass, const double RootMass, const vec2 &dist)
        {
            return -(GRAVITY_CONST * LeafMass * RootMass) / (std::pow (dist.norm (), 3)) * dist;
        }
    };
}

#endif //BARNES_HUT_DEMO_ACCELERATION_H
