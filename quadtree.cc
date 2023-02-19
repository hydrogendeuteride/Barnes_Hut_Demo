#include "quadtree.h"


Node::Node(double w, double h)
        : width (w), height (h)
{}

Node::Node(double w, double h, double BoxX, double BoxY)
        : width (w), height (h), BoxPosX (BoxX), BoxPosY (BoxY)
{}

bool Node::Find(const Body &body)
{
    if (std::find (this->data.begin (), this->data.end (), body) == this->data.end ())
        return false;
    else
        return true;
}

bool Node::Includes(const Body &body) const
{
    return (body.pos (0) > BoxPosX && body.pos (0) < BoxPosX + width) &&
           (body.pos (1) > BoxPosY && body.pos (1) < BoxPosY + height);
}

void QuadTree::Insert(const Body &body)
{
    Insert (root, body);
}

void QuadTree::Insert(const std::shared_ptr<Node> &node, const Body &body)
{
    if (Includes (body))
        node->InsertData (body);
    else
        return;

    if (body.pos (0) < (node->BoxPosX + (node->width / 2.0)))
    {
        if (body.pos (1) < (node->BoxPosY) + (node->height / 2.0))
        {
            if (node->DataCount () > 1)
            {
                if (q1 == nullptr)
                    node->q1 = std::make_shared<Node> (node->width / 2, node->height / 2,
                                                       node->BoxPosX, node->BoxPosY);
                Insert (node->q1, body);
            }
            else
                return;
        }
        else
        {
            if (node->DataCount () > 1)
            {
                if (q3 == nullptr)
                    node->q3 = std::make_shared<Node> (node->width / 2, node->height / 2,
                                                       node->BoxPosX,
                                                       node->BoxPosY + node->height / 2);
                Insert (node->q3, body);
            }
            else
                return;
        }
    }
    else
    {
        if (body.pos (1) < (node->BoxPosY) + (node->height / 2.0))
        {
            if (node->DataCount () > 1)
            {
                if (q2 == nullptr)
                    node->q2 = std::make_shared<Node> (node->width / 2, node->height / 2,
                                                       node->BoxPosX + node->width / 2,
                                                       node->BoxPosY);
                Insert (node->q2, body);
            }
            else
                return;
        }
        else
        {
            if (node->DataCount () > 1)
            {
                if (q4 == nullptr)
                    node->q4 = std::make_shared<Node> (node->width / 2, node->height / 2,
                                                       node->BoxPosX + node->width / 2,
                                                       node->BoxPosY + node->height / 2);
                Insert (node->q4, body);
            }
            else
                return;
        }
    }

}
