#include <string>
#include <iostream>
#include <vector>
#include <memory>
#include <set>
#include <SDL.h>
#include <random>
#include <algorithm>
#include <ostream>

const int block_size = 10;

class Point;
class Bag;
class Edge;

class Point {
    friend std::ostream &operator<<(std::ostream &, const Point &);
public:
    Point(int x, int y);
    void setParent(Bag *b);
    Bag *parent();
private:
    int x;
    int y;
    Bag *bag;
};

class Bag {
public:
    Bag(Point *p);
    void merge(Bag *other);
private:
    std::vector<Point *> points;
};

class Edge {
public:
    Edge(Point *p1, Point *p2);
    std::pair<Point *, Point *> getPoints();
private:
    Point *p1;
    Point *p2;
};

Edge::Edge(Point *p1, Point *p2)
    : p1(p1)
    , p2(p2)
{}

std::pair<Point *, Point *> Edge::getPoints() {
    return { p1, p2 };
}
Bag::Bag(Point *p)
    : points{ p, }
{
    p->setParent(this);
}

void Bag::merge(Bag *other) {
    for (Point *p : other->points) {
        p->setParent(this);
        points.push_back(p);
    }
}

Point::Point(int x, int y) 
    : x(x)
    , y(y)
    , bag()
{}
    
void Point::setParent(Bag *b) {
    bag = b;
}

Bag *Point::parent() {
    return bag;
}


std::ostream &operator<<(std::ostream &os, const Point &p) {
    return os << "[" << p.x << ", " << p.y << "]";
}

int main() {
    int edges;
    std::cout << "How many edges you want to generate? " << std::endl;
    std::cin >> edges;

    // Kruskal's Algorithm
    std::set<Bag *> bags;
    std::vector<Point *> points;
    std::vector<std::vector<Point *>> 
        points_map(edges, std::vector<Point *>(edges));
    std::vector<Edge *> edges_vec;

    for (int x = 0; x < edges; ++x) {
        for (int y = 0; y < edges; ++y) {
            Point *point = new Point(x, y);
            Bag *bag = new Bag(point);
            bags.insert(bag);
            points.push_back(point);
            points_map[x][y] = point;
            if (x != 0) {
                Edge *edge = 
                    new Edge(point, points_map[x - 1][y]);
                edges_vec.push_back(edge);
            }
            if (y != 0) {
                Edge *edge = 
                    new Edge(point, points_map[x][y - 1]);
                edges_vec.push_back(edge);
            }
        }
    }
    
    // 
    // shuffer the edge array
    //
    std::shuffle(edges_vec.begin(), edges_vec.end(), std::default_random_engine());
    auto it = edges_vec.begin();
    while (bags.size() != 1) {
        std::pair<Point *, Point *> p 
                = (*it)->getPoints();
        if (p.first->parent() == p.second->parent()) {
            ;
        } else {
            Bag *lhs = p.first->parent();
            Bag *rhs = p.second->parent();

            // add rhs to lhs
            lhs->merge(rhs);

            // remove rhs
            bags.erase(rhs);
            delete rhs;
        }
        ++it;
    }

    for (Edge *edge : edges_vec) {
        auto points = edge->getPoints();
        std::cout << *points.first << ", " << *points.second << std::endl;
    }

    //
    // delete 
    //
    for (Point *point : points) {
        delete point;
    }
    for (Bag *bag : bags) {
        delete bag;
    }
    for (Edge *edge : edges_vec) {
        delete edge;
    }
}
