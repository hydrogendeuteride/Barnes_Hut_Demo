#ifndef BARNES_HUT_DEMO_DRAWABLE_H
#define BARNES_HUT_DEMO_DRAWABLE_H

#include <SFML/Graphics.hpp>
#include <vector>
#include <eigen3/Eigen/Dense>

typedef Eigen::Vector2d vec2;

namespace Object
{
    class Particles
    {
    private:
        std::vector<sf::Vertex> Points;

    public:
        Particles(const std::vector<vec2> &pos)
        {
            for (const auto &x: pos)
                Points.emplace_back (sf::Vector2f (static_cast<float>(x (0)), static_cast<float>(x (1))),
                                     sf::Color::White);
        };

        void Update(const std::vector<vec2> &pos)
        {
            for (unsigned int i = 0; i < Points.size (); ++i)
            {
                Points.at (i).position = sf::Vector2f (static_cast<float>(pos.at (i) (0)),
                                                       static_cast<float>(pos.at (i) (1)));
            }
        }

        void Draw(sf::RenderWindow *window)
        {
            for (const auto &x: Points)
                window->draw (&x, 1, sf::Points);
        }
    };
}

#endif //BARNES_HUT_DEMO_DRAWABLE_H
