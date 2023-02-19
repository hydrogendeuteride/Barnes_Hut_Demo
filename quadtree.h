#ifndef BARNES_HUT_DEMO_QUADTREE_H
#define BARNES_HUT_DEMO_QUADTREE_H

#include <memory>
#include <eigen3/Eigen/Dense>
#include <utility>
#include <vector>
#include <algorithm>
#include <stack>
#include <numeric>

typedef Eigen::Vector2d vec2;
constexpr double PRECISION = 1e-15;

struct Body
{
    vec2 pos;
    vec2 vel;
    double mass;

    Body(vec2 p, vec2 v, double m) : pos (std::move(p)), vel (std::move(v)), mass (m)
    {};

    bool operator==(const Body& x)const
    {
        return (pos.isApprox (x.pos, PRECISION)) &&
                (vel.isApprox (x.vel, PRECISION)) &&
                (abs((mass - x.mass)) <= PRECISION);
    }
};

class Node
{
protected:
    std::vector<Body> data;

    bool Includes(const Body& body) const;

public:
    std::shared_ptr<Node> q1;
    std::shared_ptr<Node> q2;
    std::shared_ptr<Node> q3;
    std::shared_ptr<Node> q4;

    vec2 CenterOfMass;
    double TotalMass = 0;

    double width = 0, height = 0;
    double BoxPosX = 0, BoxPosY = 0;

    bool HasLeaf = false;

    Node(double w, double h);
    Node(double w, double h, double BoxX, double BoxY);

    bool Find(const Body& body);

    void InsertData(const Body& body)
    {
        data.push_back (body);
    }

    int DataCount()
    {
        return data.size();
    }
};

class QuadTree : public Node
{
protected:
    std::shared_ptr<Node> root;

    void Insert(const std::shared_ptr<Node>& node, const Body& body);

public:
    QuadTree(double w, double h, double boxX, double boxY) : Node(w, h, boxX, boxY)
    {
        root = std::make_shared<Node>(w, h, boxX, boxY);
    };

    void Insert(const Body& body);
};

#endif //BARNES_HUT_DEMO_QUADTREE_H