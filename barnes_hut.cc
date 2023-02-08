#include "barnes_hut.h"

vec2 BarnesHutTree::NetAcceleration(Body &leaf)
{
    vec2 NetAcc = vec2 (0.0, 0.0);

    std::queue<std::shared_ptr<Node>> queue;
    queue.push (root);

    while (!queue.empty ())
    {
        std::shared_ptr<Node> tmp = queue.front ();
        queue.pop ();

        vec2 Dist_V = leaf.pos - root->CenterOfMass;
        double Dist = Dist_V.norm ();

        if (tmp->width / Dist <= THETA || !tmp->HasLeaf)
            if (!tmp->contains (leaf))
                NetAcc += gravity (leaf.mass, tmp->TotalMass, Dist_V);

        if (tmp->q1 != nullptr)
            queue.push (tmp->q1);

        if (tmp->q2 != nullptr)
            queue.push (tmp->q2);

        if (tmp->q3 != nullptr)
            queue.push (tmp->q3);

        if (tmp->q4 != nullptr)
            queue.push (tmp->q4);
    }

    return NetAcc;
}

void BarnesHutTree::BoundaryDetection(Body &body)
{
    if ((body.pos(0) > root->width && body.vel(0) > 0) ||
        (body.pos(0) < 0 && body.vel(0) < 0))
        body.pos(0) = -body.pos(0);

    if ((body.pos(1) > root->height && body.vel(1) > 0) ||
        (body.pos(1) < 0 && body.vel(1) < 0))
        body.pos(1) = -body.pos(1);
}

void BarnesHutTree::CalcMovement(Body &body, double timestep)
{
    Integrator::Semi_Implicit_Euler euler;

    auto [x, v] =
            euler (std::make_tuple (body.pos, body.vel),
                   NetAcceleration (body), timestep);

    body.pos = x, body.vel = v;
}