#include <random>
#include <iostream>
#include <chrono>
#include "barnes_hut.h"
#include "Drawable.h"
#include "distribution.h"
#include <cmath>
#include <omp.h>

void postorder(const std::shared_ptr<Node> &Node)
{
    if (Node == nullptr)
        return;

    postorder(Node->q1);
    postorder(Node->q2);
    postorder(Node->q3);
    postorder(Node->q4);

    std::cout << Node->Width << " " << Node->Height
              << " " << Node->Data.size() << std::endl;
}

int main()
{
    std::vector<Body> bodies;

    //DiskDistribution (bodies, 1000, 20000, 50, 1, 2, 1000000);
    PlummerDistribution(bodies, 1000, 10,
                        SimWidth / 2.0, SimHeight / 2.0);

    BarnesHutTree tree = BarnesHutTree();

    sf::RenderWindow window(sf::VideoMode(ViewWidth, ViewHeight),
                            "Barnes-Hut");
    window.setVerticalSyncEnabled(true);

    Event::View view(&window, SimWidth, SimHeight, ViewWidth, ViewHeight);

    Object::Particles particles(bodies);
    Integrator::Semi_Implicit_Euler euler;

    while (window.isOpen())
    {
        sf::Event event{};
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                if (event.type == sf::Event::Closed)
                    window.close();
            }

            view.ViewTrsnsform(&window, &event);
        }
        window.clear();

        auto start = std::chrono::high_resolution_clock::now();

        tree.AddNodeIterative(bodies, SimWidth, SimHeight);

        auto middle = std::chrono::high_resolution_clock::now();
        auto treegen =
                std::chrono::duration_cast<std::chrono::milliseconds>(middle - start);

#pragma omp parallel for num_threads(omp_get_max_threads())
        for (size_t i = 0; i < bodies.size(); ++i)
        {
            tree.CalcMovement(euler, bodies.at(i), 1.0);
            tree.BoundaryDetection(bodies.at(i));
        }

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration =
                std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

        particles.Update(bodies);
        particles.Draw(&window);

        tree.Reset();

        window.display();
        std::cout << "\t" << treegen.count() << "\t" << duration.count() << "\n";
    }

    return 0;
}