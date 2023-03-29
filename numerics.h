#ifndef BARNES_HUT_DEMO_NUMERICS_H
#define BARNES_HUT_DEMO_NUMERICS_H

#include <eigen3/Eigen/Dense>
#include <tuple>

typedef Eigen::Vector2d vec2;

namespace Integrator
{
    class Semi_Implicit_Euler
    {
    public:
        std::tuple<vec2, vec2> operator()(
                const std::tuple<vec2, vec2> &Pos_Vel,
                const vec2 &accel, const double dt)
        {
            auto [x, v] = Pos_Vel;
            vec2 x_1(0.0, 0.0), v_1(0.0, 0.0);

            v_1 = v + accel * dt;
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
}

#endif //BARNES_HUT_DEMO_NUMERICS_H
