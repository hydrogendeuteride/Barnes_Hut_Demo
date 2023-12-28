#include "barnes_hut.h"

vec2 NetAcceleration(Body &leaf, const std::shared_ptr<Node> &root)
{
    Acceleration::Gravitational Gravity;

    vec2 NetAcc = vec2(0.0, 0.0);

    std::stack<std::shared_ptr<Node>> stack;
    stack.push(root);

    while (!stack.empty())  //simple DFS, in this case DFS is 50% faster than BFS
    {
        auto tmp = stack.top();
        stack.pop();

        vec2 Dist_V = leaf.pos - tmp->CenterOfMass;
        double Dist = Dist_V.norm();

        if (tmp->Width / Dist <= THETA || !tmp->HasLeaf)
        {
            if (!tmp->Contains(leaf))
                NetAcc += Gravity(tmp->TotalMass, Dist_V);
        }

        else
        {
            if (tmp->q1 != nullptr)
                stack.push(tmp->q1);

            if (tmp->q2 != nullptr)
                stack.push(tmp->q2);

            if (tmp->q3 != nullptr)
                stack.push(tmp->q3);

            if (tmp->q4 != nullptr)
                stack.push(tmp->q4);
        }
    }

    return NetAcc;
}

bool BoundaryDetection(Body &body, const std::shared_ptr<Node> &root)
{
    bool out_of_bounds = false;

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

    if (body.pos(0) < 0 || body.pos(0) > root->Width ||
        body.pos(1) < 0 || body.pos(1) > root->Height)
        out_of_bounds = true;

    return out_of_bounds;
}

void CalcMovement(Body &body, const std::shared_ptr<Node> &root, double damping, double dt,
                  Integrator::IntrgratorBase &Int)
{
    auto [x, v] =
            Int(std::make_tuple(body.pos, body.vel),
                NetAcceleration, body, root, dt);

    bool out_of_bounds = BoundaryDetection(body, root);

    if (out_of_bounds)
    {
        x = vec2(root->Width / 2 + 50.0, root->Height / 2 + 50.0);
        v = vec2(0.0, .0);
    }

    body.pos = x;
    body.vel = v * damping;
}

void DirectMethod(std::vector<Body> &bodies, double damping, double dt,
                  Integrator::IntrgratorBase &Int)
//naive calculation for debugging
{
    Acceleration::Gravitational Gravity;
    std::vector<Body> old_state = bodies;
    std::vector<Body> new_state = bodies;

    for (int i = 0; i < old_state.size(); ++i)
    {
        vec2 acc(0.0, 0.0);
        for (int j = 0; j < old_state.size(); ++j)
        {
            if (i != j)
            {
                acc += Gravity(old_state[j].mass, old_state[i].pos - old_state[j].pos);
            }
        }
        new_state[i].pos += new_state[i].vel * dt + 0.5 * acc * dt * dt;

        vec2 acc_new(0.0, 0.0);
        for (int j = 0; j < old_state.size(); ++j)
        {
            if (i != j)
            {
                acc_new += Gravity(old_state[j].mass, new_state[i].pos - old_state[j].pos);
            }
        }

        new_state[i].vel += 0.5 * (acc + acc_new) * dt;
    }

    bodies = new_state;
}