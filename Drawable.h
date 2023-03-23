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
        explicit Particles(const std::vector<Body> &pos)
        {
            for (const auto &x: pos)
                Points.emplace_back(sf::Vector2f(static_cast<float>(x.pos(0)), static_cast<float>(x.pos(1))),
                                    sf::Color::White);
        };

        void Update(const std::vector<Body> &pos)
        {
            for (unsigned int i = 0; i < Points.size(); ++i)
            {
                Points.at(i).position = sf::Vector2f(static_cast<float>(pos.at(i).pos(0)),
                                                     static_cast<float>(pos.at(i).pos(1)));
            }
        }

        void Draw(sf::RenderWindow *window)
        {
            for (const auto &x: Points)
                window->draw(&x, 1, sf::Points);
        }
    };
}

namespace Event
{
    class View
    {
    private:
        sf::View view;
        float zoom = 1.0;

    public:
        View(sf::RenderWindow *window, float SimWidth, float SimHeight, float ScreenWidth, float ScreenHeight)
        {
            view = sf::View(sf::FloatRect(SimWidth / 2 - ScreenWidth / 2,
                                          SimHeight / 2 - ScreenHeight / 2,
                                          ScreenWidth, ScreenHeight));
            view.zoom(50.0);
            zoom = 50.0;
            window->setView(view);
        }

        void ViewTrsnsform(sf::RenderWindow *window, sf::Event *event)
        {
            if (event->type == sf::Event::MouseWheelScrolled)
            {
                zoom *= 1 + (static_cast<float>(-event->mouseWheelScroll.delta) / 10);
                view.zoom(1 + static_cast<float>(-event->mouseWheelScroll.delta) / 10);
            }

            if (event->type == sf::Event::KeyPressed)
            {
                if (event->key.code == sf::Keyboard::Left)
                    view.move(-15 * zoom, 0);

                if (event->key.code == sf::Keyboard::Right)
                    view.move(15 * zoom, 0);

                if (event->key.code == sf::Keyboard::Up)
                    view.move(0, -15 * zoom);

                if (event->key.code == sf::Keyboard::Down)
                    view.move(0, 15 * zoom);
            }

            window->setView(view);
        }
    };
}

#endif //BARNES_HUT_DEMO_DRAWABLE_H
