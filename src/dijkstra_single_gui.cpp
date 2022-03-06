#include <string>
#include <iostream>
#include <vector>
#include <memory>
#include <set>
#include <SDL.h>
#include <random>
#include <algorithm>
#include <ostream>

const int block_size = 20;

class Point;
class Bag;
class Edge;

class Point {
    friend std::ostream &operator<<(std::ostream &, const Point &);
public:
    Point(int x, int y);
    void setParent(Bag *b);
    Bag *parent();
    void xy(int *x, int *y);
    void draw(SDL_Renderer *renderer);
private:
    int x;
    int y;
    Bag *bag;
};

class Bag {
public:
    Bag(Point *p);
    void merge(Bag *other);
    bool more_than_one_children();
private:
    std::vector<Point *> points;
};

class Edge {
public:
    Edge(Point *p1, Point *p2);
    std::pair<Point *, Point *> getPoints();
    void draw(SDL_Renderer *renderer);
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


void Point::xy(int *x, int *y) {
    *x = this->x;
    *y = this->y;
}

bool Bag::more_than_one_children() {
    return this->points.size() > 1;
}


void Point::draw(SDL_Renderer *renderer) {
    if (this->bag->more_than_one_children()) {
        SDL_SetRenderDrawColor(renderer, 0xff, 0xff, 0xff, 0xff);
    } else {
        SDL_SetRenderDrawColor(renderer, 0xaa, 0xaa, 0xaa, 0xff);
    }
    SDL_Rect rect{x * block_size, y * block_size, block_size, block_size};
    SDL_RenderFillRect(renderer, &rect);
}

void Edge::draw(SDL_Renderer *renderer) {
    //
    // Set the color black;
    //
    SDL_SetRenderDrawColor(renderer, 0x0, 0x0, 0x0, 0xff);
    int x1, x2, y1, y2;
    p1->xy(&x1, &y1);
    p2->xy(&x2, &y2);

    if (x1 == x2) {
        int x = x1; 
        int y = std::max(y1, y2);
        SDL_RenderDrawLine(renderer, x * block_size, y * block_size, 
                           (x + 1) * block_size, y * block_size);
    } else {
        int x = std::max(x1, x2);
        int y = y1;
        SDL_RenderDrawLine(renderer, x * block_size, y * block_size, 
                           x * block_size, (y + 1) * block_size);
    }
}

void draw_map(SDL_Renderer *renderer, std::set<Edge *> edges, std::vector<Point *> points) {
    SDL_SetRenderDrawColor(renderer, 0xff, 0xff, 0xff, 0xff);
    //
    // Set the map white
    //
    SDL_RenderFillRect(renderer, 0x0);
    for (Point *point : points) {
        point->draw(renderer);
    }
    for (Edge *edge : edges) {
        edge->draw(renderer);
    }
    SDL_RenderPresent(renderer);
}

int main() {
    int edges;
    std::cout << "How many edges you want to generate? " << std::endl;
    std::cin >> edges;

    // 
    // Init SDL 
    //
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) != 0) {
        SDL_Log("Unable to initialize SDL: %s", SDL_GetError());
        return 1;
    }

    SDL_Window *window;
    SDL_Renderer *renderer;

    // 
    // Create window and renderer
    //
    
    // Create an application window with the following settings:
    window = SDL_CreateWindow(
        "An SDL2 window",                  // window title
        SDL_WINDOWPOS_UNDEFINED,           // initial x position
        SDL_WINDOWPOS_UNDEFINED,           // initial y position
        edges * block_size,                               // width, in pixels
        edges * block_size,                               // height, in pixels
        SDL_WINDOW_OPENGL                  // flags - see below
    );

    renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);


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

    std::set<Edge *> edges_to_draw(edges_vec.begin(), edges_vec.end());
    // 
    // draw the map 
    //
    draw_map(renderer, edges_to_draw, points);
    
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

            edges_to_draw.erase(*it);
        }
        ++it;

        SDL_Delay(8);

        draw_map(renderer, edges_to_draw, points);
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
    // delay 3 s
    SDL_Delay(3000);

    // 
    // Destory window and renderer
    //
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);

    // quit SDL 
    SDL_Quit();

}
