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
    void CalcMovement(Body &body, double dt);

    void BoundaryDetection(Body &body);
};

vec2 NetAcceleration(Body &leaf, const std::shared_ptr<Node>& root);
void BoundaryDetection(Body &body, const std::shared_ptr<Node>& root);
void CalcMovement(Body &body, const std::shared_ptr<Node>& root, double dt);

#endif //BARNES_HUT_DEMO_BARNES_HUT_H