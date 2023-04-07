#include "barnes_hut.h"

vec2 NetAcceleration(Body &leaf, const std::shared_ptr<Node> &root)
{
    Acceleration::Gravitational Gravity;

    vec2 NetAcc = vec2(0.0, 0.0);

    std::stack<std::shared_ptr<Node>> stack;
    stack.push(root);

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

void BoundaryDetection(Body &body, const std::shared_ptr<Node> &root)
{
    if (body.pos(0) > root->Width && body.vel(0) > 0)
    {
        body.vel(0) = -body.vel(0);
        body.pos(0) = root->Width;
    }
    if (body.pos(0) < 0 && body.vel(0) < 0)
    {
        body.vel(0) = -body.vel(0);
        body.pos(0) = 0.0;
    }
    if (body.pos(1) > root->Height && body.vel(1) > 0)
    {
        body.vel(1) = -body.vel(1);
        body.pos(1) = root->Height;
    }
    if (body.pos(1) < 0 && body.vel(1) < 0)
    {
        body.vel(1) = -body.vel(1);
        body.pos(1) = 0.0;
    }
}

void CalcMovement(Body &body, const std::shared_ptr<Node>& root, double dt)
{
    Integrator::Velocity_Verlet Verlet;

    auto [x, v] =
            Verlet(std::make_tuple(body.pos, body.vel),
                   NetAcceleration, body, root, dt);

    BoundaryDetection(body, root);

    body.pos = x, body.vel = v;
}