#include <random>
#include <iostream>
#include <chrono>
#include "barnes_hut.h"
#include "Drawable.h"
#include <cmath>
#include <omp.h>

void postorder(const std::shared_ptr<Node> &Node)
{
    if (Node == nullptr)
        return;

    postorder (Node->q1);
    postorder (Node->q2);
    postorder (Node->q3);
    postorder (Node->q4);

    std::cout << Node->width << " " << Node->height << " " << Node->data.size () << std::endl;
}

void DiskDistribution(
        std::vector<Body> &bodies, int number, double rad_max, double rad_min,
        double mass_min, double mass_max, double centermass)
{
    //https://en.wikipedia.org/wiki/Plummer_model
    //http://www.artcompsci.org/kali/vol/plummer/volume11.pdf
    std::random_device rand;
    std::mt19937 gen (rand ());
    std::uniform_real_distribution<> rand_angle (0, 2 * M_PI);
    std::uniform_real_distribution<> rand_dist (rad_min, rad_max);
    std::uniform_real_distribution<> rand_mass (mass_min, mass_max);

    bodies.emplace_back (vec2 (SimWidth / 2.0, SimHeight / 2.0), vec2 (0.0, 0.0), centermass);

    for (int i = 1; i < number; ++i)
    {
        double angle = rand_angle (gen);
        double distance = rand_dist (gen);
        double pos_x = std::cos (angle) * distance + (ViewWidth / 2.0);
        double pos_y = std::sin (angle) * distance + (ViewHeight / 2.0);

        double velocity = std::pow (centermass * GRAVITY_CONST / distance, 0.5);
        double vel_x = std::sin (angle) * velocity;
        double vel_y = -std::cos (angle) * velocity;

        double mass = rand_mass (gen);

        bodies.emplace_back (vec2 (pos_x, pos_y), vec2 (vel_x, vel_y), mass);
    }
}

void PlummerDistribution(
        std::vector<Body> &bodies, int number, double rad_max,
        double x_center, double y_center)
{
    std::random_device rand;
    std::mt19937 gen (rand ());
    std::uniform_real_distribution<> phi_rand (0.0, 2 * M_PI);
    std::uniform_real_distribution<> norm_rand (0.0, 1.0);

    for (int i = 0; i < number; ++i)
    {
        double dist_norm = norm_rand (gen);
        double phi = phi_rand (gen);

        double r = rad_max / std::sqrt (std::pow (dist_norm, -0.66666666667) - 1.0);
        vec2 pos = vec2 (r * std::cos (phi) * r + x_center, r * std::sin (phi) * r + y_center);

        double x = 0.1;
        double y;
        while (1)
        {
            y = norm_rand (gen);
            if (y < x * x * std::pow ((1 - x * x), 1.5))
                break;
            x = norm_rand (gen);
        }

        double v = x * std::sqrt (2.0) * std::pow((1 + r * r), -0.25);
        double phi_v = 2.0 * M_PI * norm_rand(gen);

        vec2 vel = v * vec2(std::cos(phi_v), std::sin(phi_v));

        bodies.emplace_back (pos, vel, 1.0);
    }
}

int main()
{
    std::vector<Body> bodies;

    //DiskDistribution (bodies, 1000, 20000, 50, 1, 2, 1000000);
    PlummerDistribution (bodies, 1000, 10, SimWidth / 2.0, SimHeight / 2.0);

    BarnesHutTree tree = BarnesHutTree ();

    sf::RenderWindow window (sf::VideoMode (ViewWidth, ViewHeight), "Barnes-Hut");
    window.setVerticalSyncEnabled (true);

    Event::View view (&window, SimWidth, SimHeight, ViewWidth, ViewHeight);

    Object::Particles particles (bodies);

    while (window.isOpen ())
    {
        sf::Event event{};
        while (window.pollEvent (event))
        {
            if (event.type == sf::Event::Closed)
            {
                if (event.type == sf::Event::Closed)
                    window.close ();
            }

            view.ViewTrsnsform (&window, &event);
        }
        window.clear ();

        auto start = std::chrono::high_resolution_clock::now ();

        tree.addNodeIterative (bodies, SimWidth, SimHeight);

        auto middle = std::chrono::high_resolution_clock::now ();
        auto treegen = std::chrono::duration_cast<std::chrono::milliseconds> (middle - start);

#pragma omp parallel for num_threads(omp_get_max_threads())
        for (size_t i = 0; i < bodies.size (); ++i)
        {
            tree.CalcMovement (bodies.at (i), 1.0);
            tree.BoundaryDetection (bodies.at (i));
        }

        auto stop = std::chrono::high_resolution_clock::now ();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds> (stop - start);

        particles.Update (bodies);
        particles.Draw (&window);

        tree.reset ();

        window.display ();
        std::cout << "\t" << treegen.count () << "\t" << duration.count () << "\n";
    }

    return 0;
}