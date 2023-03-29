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
{/*
    Integrator::Semi_Implicit_Euler euler;

    auto [x, v] =
            euler(std::make_tuple(body.pos, body.vel),
                  NetAcceleration(body), dt);

    body.pos = x, body.vel = v;
*/
    Integrator::Verlet_First first;
    Integrator::Verlet_Last Last;

    vec2 accel = NetAcceleration(body);
    auto [x, v] =
            first(std::make_tuple(body.pos, body.vel),
                  NetAcceleration(body), dt);

    body.pos = x, body.vel = v;

    vec2 accel_1 = NetAcceleration(body);
    auto [x_1, v_1] =
            Last(std::make_tuple(body.pos, body.vel),
                  accel + accel_1, dt);

    body.pos = x_1, body.vel = v_1;

    /*
    vec2 x = body.pos, v = body.vel, a = NetAcceleration(body);
    vec2 x_1(0.0, 0.0), v_1(0.0, 0.0), a_1(0.0, 0.0);
    x_1 = x + v * dt + 0.5 * a * dt * dt;

    body.pos = x_1;
    a_1 = NetAcceleration(body);
    v_1 = v + 0.5 * (a + a_1) * dt;

    body.pos = x_1, body.vel = v_1;
*/
}