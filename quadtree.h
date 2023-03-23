#ifndef BARNES_HUT_DEMO_QUADTREE_H
#define BARNES_HUT_DEMO_QUADTREE_H

#include <memory>
#include <eigen3/Eigen/Dense>
#include <utility>
#include <vector>
#include <queue>
#include <algorithm>
#include <stack>
#include <numeric>

typedef Eigen::Vector2d vec2;
constexpr double PRECISION = 1e-10;

struct Body
{
    vec2 pos;
    vec2 vel;
    double mass;

    Body(vec2 p, vec2 v, double m) : pos(std::move(p)), vel(std::move(v)), mass(m)
    {};

    bool operator==(const Body &x) const
    {
        return (pos.isApprox(x.pos, PRECISION)) &&
               (vel.isApprox(x.vel, PRECISION)) &&
               (abs((mass - x.mass)) <= PRECISION);
    }
};

class Node
{
public:
    std::vector<Body> Data;

    std::shared_ptr<Node> q1;
    std::shared_ptr<Node> q2;
    std::shared_ptr<Node> q3;
    std::shared_ptr<Node> q4;

    vec2 CenterOfMass;
    double TotalMass = 0;

    double Width = 0, Height = 0;
    double BoxPosX = 0, BoxPosY = 0;

    bool HasLeaf = false;

    Node(std::vector<Body> data, double w, double h);

    Node(std::vector<Body> data, double w, double h, double BoxX, double BoxY);

    bool Contains(const Body &body);
};

class QuadTree
{
public:
    QuadTree() : Root(nullptr)
    {};

    void AddNodeIterative(std::vector<Body> &data, double w, double h);

    void Reset();

protected:
    std::shared_ptr<Node> Root;
};

#endif //BARNES_HUT_DEMO_QUADTREE_H