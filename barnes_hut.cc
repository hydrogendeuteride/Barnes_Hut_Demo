#include "barnes_hut.h"

vec2 BarnesHutTree::NetAcceleration(Body &leaf)
{
    vec2 NetAcc = vec2(0.0, 0.0);

    std::stack<std::shared_ptr<Node>> stack;
    stack.push(Root);

    while (!stack.empty())
    {
        auto tmp = stack.top();
        stack.pop();

        vec2 Dist_V = leaf.pos - tmp->CenterOfMass;
        double Dist = Dist_V.norm();

        if (tmp->Width / Dist <= THETA || !tmp->HasLeaf)
            if (!tmp->Contains(leaf))
                NetAcc += Gravity(tmp->TotalMass, Dist_V);

        if (tmp->q1 != nullptr)
            stack.push(tmp->q1);

        if (tmp->q2 != nullptr)
            stack.push(tmp->q2);

        if (tmp->q3 != nullptr)
            stack.push(tmp->q3);

        if (tmp->q4 != nullptr)
            stack.push(tmp->q4);
    }

    return NetAcc;
}

void BarnesHutTree::BoundaryDetection(Body &body)
{
    if ((body.pos(0) > Root->Width && body.vel(0) > 0) ||
        (body.pos(0) < 0 && body.vel(0) < 0))
        body.vel(0) = -body.vel(0);

    if ((body.pos(1) > Root->Height && body.vel(1) > 0) ||
        (body.pos(1) < 0 && body.vel(1) < 0))
        body.vel(1) = -body.vel(1);
}

void BarnesHutTree::CalcMovement(Body &body, double dt)
{
    Integrator::Semi_Implicit_Euler euler;

    auto [x, v] =
            euler(std::make_tuple(body.pos, body.vel),
                  NetAcceleration(body), dt);

    body.pos = x, body.vel = v;
}