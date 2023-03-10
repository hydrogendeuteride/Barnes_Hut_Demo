#ifndef BARNES_HUT_DEMO_BARNES_HUT_H
#define BARNES_HUT_DEMO_BARNES_HUT_H

#include "quadtree.h"
#include "numerics.h"
#include "acceleration.h"
#include "constants.h"

class BarnesHutTree : public QuadTree
{
private:
    vec2 NetAcceleration(Body &leaf);

    Acceleration::Gravitational gravity;

public:
    void CalcMovement(Body &body, double timestep);
    void BoundaryDetection(Body& body);
};

#endif //BARNES_HUT_DEMO_BARNES_HUT_H