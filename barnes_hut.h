#ifndef BARNES_HUT_DEMO_BARNES_HUT_H
#define BARNES_HUT_DEMO_BARNES_HUT_H

#include "quadtree.h"
#include "numerics.h"
#include "acceleration.h"
#include "constants.h"
#include <functional>

vec2 NetAcceleration(Body &leaf, const std::shared_ptr<Node>& root);

bool BoundaryDetection(Body &body, const std::shared_ptr<Node>& root);

void CalcMovement(Body &body, const std::shared_ptr<Node>& root, double dt);

#endif //BARNES_HUT_DEMO_BARNES_HUT_H