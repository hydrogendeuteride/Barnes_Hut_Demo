#ifndef BARNES_HUT_DEMO_PHYSICS_H
#define BARNES_HUT_DEMO_PHYSICS_H

#include <vector>
#include "quadtree.h"
#include "constants.h"

double Kinetic_Energy(const std::vector<Body> &bodies)
{
    return 0.5 * std::accumulate(bodies.begin(), bodies.end(), 0.0,
                                 [](double ke, const auto &x)
                                 { return ke + x.mass * x.vel.squaredNorm(); });
}

double Potential_Energy(const std::vector<Body> &bodies)    //O(n^2) but fast
{
    double sum = 0;
    for (int i = 0; i < bodies.size(); ++i)
    {
        for (int j = 0; j < i; ++j)
        {
            sum += bodies[i].mass * bodies[j].mass / (bodies[j].pos - bodies[i].pos).norm();
        }
    }

    return -GRAVITY_CONST * sum;
}

double Virial_Energy(const std::vector<Body> &bodies)
{
    double kinetic_energy = Kinetic_Energy(bodies);
    double potential_energy = Potential_Energy(bodies);
    return 2.0 * kinetic_energy - potential_energy;
}

double Total_Energy(const std::vector<Body> &bodies)
{
    double kinetic_energy = Kinetic_Energy(bodies);
    double potential_energy = Potential_Energy(bodies);
    return kinetic_energy + potential_energy;
}

#endif //BARNES_HUT_DEMO_PHYSICS_H
