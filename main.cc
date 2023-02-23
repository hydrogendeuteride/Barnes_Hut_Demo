#include <iostream>
#include <random>
#include <chrono>
#include "barnes_hut.h"
#include "Drawable.h"

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

int main()
{
    std::random_device rand;
    std::mt19937 gen (rand ());
    std::uniform_real_distribution<> dist (0.0, 10.0);
    std::uniform_real_distribution<> mass (1.0, 2.0);

    std::vector<Body> bodies;
    for (int i = 0; i < 1000; ++i)
        bodies.emplace_back (vec2 (dist (gen), dist (gen)),
                             vec2 (dist (gen), dist (gen)),
                             mass (gen));

    BarnesHutTree tree = BarnesHutTree ();

    sf::RenderWindow window(sf::VideoMode(1920, 1080), "Barnes-Hut");
    window.setVerticalSyncEnabled (true);

    Object::Particles particles(bodies);

    while (window.isOpen())
    {
        sf::Event event{};
        while (window.pollEvent (event))
        {
            if (event.type == sf::Event::Closed)
            {
                if (event.type == sf::Event::Closed)
                    window.close();
            }
        }
        window.clear();

        auto start = std::chrono::high_resolution_clock::now ();

        tree.addNodeIterative (bodies, 50, 50);

        auto middle = std::chrono::high_resolution_clock::now ();
        auto treegen = std::chrono::duration_cast<std::chrono::milliseconds> (middle - start);

        for (auto &x: bodies)
        {
            tree.CalcMovement (x, 0.0001);
            tree.BoundaryDetection (x);
        }

        auto stop = std::chrono::high_resolution_clock::now ();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds> (stop - start);

        particles.Update (bodies);
        particles.Draw (&window);

        tree.reset();

        //std::cout << i << "\t" << treegen.count () << "\t" << duration.count () << "\n";
    }

    return 0;
}