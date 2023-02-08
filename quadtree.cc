#include "quadtree.h"


Node::Node(std::vector<Body> data, double w, double h)
        : data (std::move (data)), width (w), height (h)
{
    TotalMass = std::accumulate (this->data.begin (), this->data.end (), 0,
                                 [](double sum, const auto &x) { return sum + x.mass; });

    CenterOfMass = std::accumulate (this->data.begin (), this->data.end (), vec2 (0.0, 0.0),
                                    [](const vec2 &sum, const auto &x) { return sum + (x.mass * x.pos); }) / TotalMass;
}

Node::Node(std::vector<Body> data, double w, double h, double BoxX, double BoxY)
        : data (std::move (data)), width (w), height (h), BoxPosX (BoxX), BoxPosY (BoxY)
{
    TotalMass = std::accumulate (this->data.begin (), this->data.end (), 0,
                                 [](double sum, const auto &x) { return sum + x.mass; });

    CenterOfMass = std::accumulate (this->data.begin (), this->data.end (), vec2 (0.0, 0.0),
                                    [](const vec2 &sum, const auto &x) { return sum + (x.mass * x.pos); }) / TotalMass;
}

bool Node::contains(const Body &body)
{

    if (std::find (this->data.begin (), this->data.end (), body) == this->data.end ())
        return false;
    else
        return true;
}

void QuadTree::reset()
{
    std::stack<std::shared_ptr<Node>> stack1, stack2;
    std::shared_ptr<Node> tmp;
    stack1.push (root);

    /*
     *  leaf4 -> pop
     *  leaf3                    (leaf3)
     *  leaf2                    (leaf4)
     *  leaf1     -(leaf4)->      root
     *  ______                    ______
     *  stack1                    stack2
     */

    while (!stack1.empty ())
    {
        tmp = stack1.top ();
        stack1.pop ();
        stack2.push (tmp);

        if (tmp->q1 != nullptr)
            stack1.push (tmp->q1);

        if (tmp->q2 != nullptr)
            stack1.push (tmp->q2);

        if (tmp->q3 != nullptr)
            stack1.push (tmp->q3);

        if (tmp->q4 != nullptr)
            stack1.push (tmp->q4);
    }

    while (!stack2.empty())
    {
        tmp = stack2.top();
        stack2.pop();

        tmp->HasLeaf = false;
        tmp.reset();
    }

    root.reset();
}

void QuadTree::addNodeIterative(std::vector<Body> &data, double w, double h)
{
    std::queue<std::shared_ptr<Node>> queue;

    if (root != nullptr)
    {
        root->HasLeaf = true;
        queue.push (root);
    }
    else
    {
        root = std::make_shared<Node> (data, w, h, 0, 0);
        root->HasLeaf = true;
        queue.push (root);
    }

    while (!queue.empty ())
    {
        std::shared_ptr<Node> tmp = queue.front ();
        queue.pop ();

        std::vector<Body> q1, q2, q3, q4;


        for (auto &body1: tmp->data)
        {
            if (body1.pos (0, 0) < (tmp->BoxPosX + (tmp->width / 2.0)))
            {
                if (body1.pos (1, 0) < (tmp->BoxPosY) + (tmp->height / 2.0))
                    q1.push_back (body1);
                else
                    q3.push_back (body1);
            }
            else
            {
                if (body1.pos (1, 0) < (tmp->BoxPosY) + (tmp->height / 2.0))
                    q2.push_back (body1);
                else
                    q4.push_back (body1);
            }
        }
        /*
         *0  1  2  3  4  5  6  7
         *1           |
         *2     q1    |     q2
         *3           |
         *4-----------|--------------
         *5           |
         *6     q3    |     q4
         *7           |
         */

        if (!q1.empty ())
        {
            tmp->q1 = std::make_shared<Node> (q1, tmp->width / 2.0, tmp->height / 2.0,
                                              tmp->BoxPosX, tmp->BoxPosY);

            if (q1.size () > 1)
            {
                queue.push (tmp->q1);
                tmp->HasLeaf = true;
            }
        }
        if (!q2.empty ())
        {
            tmp->q2 = std::make_shared<Node> (q2, tmp->width / 2.0, tmp->height / 2.0,
                                              tmp->BoxPosX + tmp->width / 2.0, tmp->BoxPosY);

            if (q2.size () > 1)
            {
                queue.push (tmp->q2);
                tmp->HasLeaf = true;
            }
        }
        if (!q3.empty ())
        {
            tmp->q3 = std::make_shared<Node> (q3, tmp->width / 2.0, tmp->height / 2.0,
                                              tmp->BoxPosX, tmp->BoxPosY + tmp->height / 2.0);

            if (q3.size () > 1)
            {
                queue.push (tmp->q3);
                tmp->HasLeaf = true;
            }
        }
        if (!q4.empty ())
        {
            tmp->q4 = std::make_shared<Node> (q4, tmp->width / 2.0, tmp->height / 2.0,
                                              tmp->BoxPosX + tmp->width / 2.0, tmp->BoxPosY + tmp->height / 2.0);

            if (q4.size () > 1)
            {
                queue.push (tmp->q4);
                tmp->HasLeaf = true;
            }
        }
    }
}