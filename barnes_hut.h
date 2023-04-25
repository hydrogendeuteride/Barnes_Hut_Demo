#ifndef BARNES_HUT_DEMO_BARNES_HUT_H
#define BARNES_HUT_DEMO_BARNES_HUT_H

#include "quadtree.h"
#include "numerics.h"
#include "acceleration.h"
#include "constants.h"
#include <functional>

//tree traversal time complexity O(n(log(n))
vec2 NetAcceleration(Body &leaf, const std::shared_ptr<Node>& root);

//boundary particle detection O(n)
bool BoundaryDetection(Body &body, const std::shared_ptr<Node>& root);

//integrator, acceleration merge function
void CalcMovement(Body &body, const std::shared_ptr<Node>& root, double damping, double dt, Integrator::IntrgratorBase &Int);

#endif //BARNES_HUT_DEMO_BARNES_HUT_H