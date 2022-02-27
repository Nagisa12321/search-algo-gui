#include <string>
#include <iostream>
#include <vector>
#include <memory>
#include <set>
#include <SDL.h>
#include <random>
#include <algorithm>

const int block_size = 10;

class Point;
class Bag;
class Edge;

class Point {
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
    Bag(std::shared_ptr<Point> p);
    void merge(std::shared_ptr<Bag> other);
private:
    std::vector<std::shared_ptr<Point>> points;
};

class Edge {
public:
    Edge(std::shared_ptr<Point> p1, std::shared_ptr<Point> p2);
    std::pair<std::shared_ptr<Point>, std::shared_ptr<Point>> getPoints();
private:
    std::shared_ptr<Point> p1;
    std::shared_ptr<Point> p2;
};

Edge::Edge(std::shared_ptr<Point> p1, std::shared_ptr<Point> p2)
    : p1(p1)
    , p2(p2)
{}

std::pair<std::shared_ptr<Point>, std::shared_ptr<Point>> Edge::getPoints() {
    return { p1, p2 };
}
Bag::Bag(std::shared_ptr<Point> p)
    : points{ p, }
{
    p->setParent(this);
}

void Bag::merge(std::shared_ptr<Bag> other) {
    for (std::shared_ptr<Point> p : other->points) {
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

int main() {
    int edges;
    std::cout << "How many edges you want to generate? " << std::endl;
    std::cin >> edges;

    // Kruskal's Algorithm
    std::set<std::shared_ptr<Bag>> bags;
    std::vector<std::shared_ptr<Point>> points;
    std::vector<std::vector<std::shared_ptr<Point>>> 
        points_map(edges, std::vector<std::shared_ptr<Point>>(edges));
    std::vector<std::shared_ptr<Edge>> edges_vec;

    for (int x = 0; x < edges; ++x) {
        for (int y = 0; y < edges; ++y) {
            std::shared_ptr<Point> point = std::make_shared<Point>(x, y);
            std::shared_ptr<Bag> bag = std::make_shared<Bag>(point);
            bags.insert(bag);
            points.push_back(point);
            points_map[x][y] = point;
            if (x != 0) {
                std::shared_ptr<Edge> edge = 
                    std::make_shared<Edge>(point, points_map[x - 1][y]);
                edges_vec.push_back(edge);
            }
            if (y != 0) {
                std::shared_ptr<Edge> edge = 
                    std::make_shared<Edge>(point, points_map[x][y - 1]);
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
        std::pair<std::shared_ptr<Point>, std::shared_ptr<Point>> p 
                = (*it)->getPoints();
        if (p.first->parent() == p.second->parent()) {
            ;
        } else {
            std::shared_ptr<Bag> other(p.second->parent());
            p.first->parent()->merge(other);
            bags.erase(other);
        }
    }
}
