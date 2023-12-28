#ifndef BARNES_HUT_DEMO_NUMERICS_H
#define BARNES_HUT_DEMO_NUMERICS_H

#include <eigen3/Eigen/Dense>
#include <tuple>

typedef Eigen::Vector2d vec2;

namespace Integrator
{
    class IntrgratorBase    //all integrator should follow this form
    {
    public:
        virtual std::tuple<vec2, vec2> operator()(
                const std::tuple<vec2, vec2> &Pos_Vel,
                const std::function<vec2(Body &leaf, const std::shared_ptr<Node> &root)> &Acc,
                Body &leaf, const std::shared_ptr<Node> &root, double dt) = 0;
    };

    class Semi_Implicit_Euler : public IntrgratorBase
    {
    public:
        std::tuple<vec2, vec2> operator()(
                const std::tuple<vec2, vec2> &Pos_Vel,
                const std::function<vec2(Body &leaf, const std::shared_ptr<Node> &root)> &Acc,
                Body &leaf, const std::shared_ptr<Node> &root, double dt)
        {
            auto [x, v] = Pos_Vel;
            vec2 x_1(0.0, 0.0), v_1(0.0, 0.0);

            v_1 = v + Acc(leaf, root) * dt;
            x_1 = x + v_1 * dt;

            return std::make_tuple(x_1, v_1);
        }
    };

    class Verlet_First
    {
    public:
        std::tuple<vec2, vec2> operator()(
                const std::tuple<vec2, vec2> &Pos_Vel,
                const vec2 &accel, const double dt)
        {
            auto [x, v] = Pos_Vel;

            x += v * dt + 0.5 * accel * dt * dt;

            return std::make_tuple(x, v);
        }
    };

    class Verlet_Last
    {
    public:
        std::tuple<vec2, vec2> operator()(
                const std::tuple<vec2, vec2> &Pos_Vel,
                const vec2 &accel_sum, const double dt)
        {
            auto [x, v] = Pos_Vel;

            v += 0.5 * accel_sum * dt;

            return std::make_tuple(x, v);
        }
    };

    class Velocity_Verlet : public IntrgratorBase
    {
    public:
        std::tuple<vec2, vec2> operator()(
                const std::tuple<vec2, vec2> &Pos_Vel,
                const std::function<vec2(Body &leaf, const std::shared_ptr<Node> &root)> &Acc,
                Body &leaf, const std::shared_ptr<Node> &root, double dt)
        {
            auto [x, v] = Pos_Vel;

            vec2 acc = Acc(leaf, root);
            x += v * dt + 0.5 * Acc(leaf, root) * dt * dt;
            leaf.pos = x;

            vec2 acc_new = Acc(leaf, root);
            v += 0.5 * (acc + acc_new) * dt;

            return std::make_tuple(x, v);
        }
    };
}

#endif //BARNES_HUT_DEMO_NUMERICS_H
