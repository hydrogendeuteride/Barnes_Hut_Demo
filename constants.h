#ifndef BARNES_HUT_DEMO_CONSTANTS_H
#define BARNES_HUT_DEMO_CONSTANTS_H

constexpr double SimWidth = 327680;
constexpr double SimHeight = 327680;    // main simulation size

constexpr double ViewWidth = 1920;
constexpr double ViewHeight = 1080; //main sfml window size

constexpr double THETA = 0.5;   //barnes_hut.h, tree search threshold

constexpr double EPSILON = 0.5; //acceleration.h, force softening
/*
 * smaller softening value is good for energy conservation
 */

constexpr double GRAVITY_CONST = 0.1; // acceleration.h, gravity constant

#endif //BARNES_HUT_DEMO_CONSTANTS_H
