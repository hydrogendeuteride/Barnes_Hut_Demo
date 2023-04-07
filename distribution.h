#ifndef BARNES_HUT_DEMO_DISTRIBUTION_H
#define BARNES_HUT_DEMO_DISTRIBUTION_H

void DiskDistribution(
        std::vector<Body> &bodies, int number, double rad_max, double rad_min,
        double mass_min, double mass_max, double centermass)
{
    std::random_device rand;
    std::mt19937 gen(rand());
    std::uniform_real_distribution<> rand_angle(0, 2 * M_PI);
    std::uniform_real_distribution<> rand_dist(rad_min, rad_max);
    std::uniform_real_distribution<> rand_mass(mass_min, mass_max);

    bodies.emplace_back(vec2(SimWidth / 2.0, SimHeight / 2.0),
                        vec2(0.0, 0.0), centermass);

    for (int i = 1; i < number; ++i)
    {
        double angle = rand_angle(gen);
        double distance = rand_dist(gen);
        double pos_x = std::cos(angle) * distance + (SimWidth / 2.0);
        double pos_y = std::sin(angle) * distance + (SimHeight / 2.0);

        double velocity = std::pow(centermass * GRAVITY_CONST / distance, 0.5);
        double vel_x = std::sin(angle) * velocity;
        double vel_y = -std::cos(angle) * velocity;

        double mass = rand_mass(gen);

        bodies.emplace_back(vec2(pos_x, pos_y),
                            vec2(vel_x, vel_y), mass);
    }
}

void PlummerDistribution(
        std::vector<Body> &bodies, int number, double rad_max,
        double x_center, double y_center)
{
    //https://en.wikipedia.org/wiki/Plummer_model
    //http://www.artcompsci.org/kali/vol/plummer/volume11.pdf
    std::random_device rand;
    std::mt19937 gen(rand());
    std::uniform_real_distribution<> phi_rand(0.0, 2 * M_PI);
    std::uniform_real_distribution<> norm_rand(0.0, 1.0);

    for (int i = 0; i < number; ++i)
    {
        double dist_norm = norm_rand(gen);
        double phi = phi_rand(gen);

        double r = rad_max / std::sqrt(std::pow(dist_norm, -0.66666666667) - 1.0);
        vec2 pos = vec2(r * std::cos(phi) + x_center,
                        r * std::sin(phi) + y_center);

        double x = 0.1;
        double y;
        while (true)
        {
            y = norm_rand(gen);
            if (y < x * x * std::pow((1 - x * x), 1.5))
                break;
            x = norm_rand(gen);
        }

        double v = x * std::sqrt(2.0) * std::pow((1 + r * r), -0.25);
        double phi_v = 2.0 * M_PI * norm_rand(gen);

        vec2 vel = v * vec2(std::cos(phi_v), std::sin(phi_v));

        bodies.emplace_back(pos, vel, 1.0);
    }
}

#endif //BARNES_HUT_DEMO_DISTRIBUTION_H
