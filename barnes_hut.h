#ifndef BARNES_HUT_DEMO_BARNES_HUT_H
#define BARNES_HUT_DEMO_BARNES_HUT_H

#include "quadtree.h"
#include "numerics.h"
#include "acceleration.h"
#include "constants.h"
#include <functional>

class BarnesHutTree : public QuadTree
{
private:
    vec2 NetAcceleration(Body &leaf);

    Acceleration::Gravitational Gravity;

public:
    void CalcMovement(const std::function<std::tuple<vec2, vec2>
            (const std::tuple<vec2, vec2> &Pos_Vel,
             const vec2 &accel, const double dt)>& Int, Body &body, double dt);

    void BoundaryDetection(Body &body);
    std::shared_ptr<Node> GetRoot();
};

vec2 NetAcceleration(Body &leaf, const std::shared_ptr<Node>& Root);

#endif //BARNES_HUT_DEMO_BARNES_HUT_H