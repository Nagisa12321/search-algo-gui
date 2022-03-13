#include "SDL2/SDL_events.h"
#include "SDL2/SDL_rect.h"
#include "SDL2/SDL_render.h"
#include <cmath>
#include <string>
#include <iostream>
#include <vector>
#include <memory>
#include <set>
#include <SDL2/SDL.h>
#include <random>
#include <algorithm>
#include <ostream>
#include <unordered_map>

// #define PRINT_MARTIX

const int block_size = 10;
const int red_dot_size = 8;
const int i4_huge = 0xffff;

class Point;
class Bag;
class Edge;

class Point {
    friend std::ostream &operator<<(std::ostream &, const Point &);
public:
    Point(int x, int y, int id);
    void setParent(Bag *b);
    Bag *parent() const;
    void xy(int *x, int *y) const;
    int getId() const;
    void draw(SDL_Renderer *renderer) const;
    void drawRedDot(SDL_Renderer *renderer) const;
    void center(int *x, int *y) const;
private:
    int x;
    int y;
    Bag *bag;
    int id;
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

Point::Point(int x, int y, int id) 
    : x(x)
    , y(y)
    , bag()
    , id(id)
{}
    
void Point::setParent(Bag *b) {
    bag = b;
}

Bag *Point::parent() const {
    return bag;
}


std::ostream &operator<<(std::ostream &os, const Point &p) {
    return os << "[" << p.x << ", " << p.y << "]";
}


void Point::xy(int *x, int *y) const {
    *x = this->x;
    *y = this->y;
}

bool Bag::more_than_one_children() {
    return this->points.size() > 1;
}

int Point::getId() const {
    return id;
}

void Point::center(int *x, int *y) const {
    *x = this->x * block_size + block_size / 2;
    *y = this->y * block_size + block_size / 2;
}

void Point::draw(SDL_Renderer *renderer) const {
    if (this->bag->more_than_one_children()) {
        SDL_SetRenderDrawColor(renderer, 0xff, 0xff, 0xff, 0xff);
    } else {
        SDL_SetRenderDrawColor(renderer, 0xaa, 0xaa, 0xaa, 0xff);
    }
    SDL_Rect rect{x * block_size, y * block_size, block_size, block_size};
    SDL_RenderFillRect(renderer, &rect);
}

void Point::drawRedDot(SDL_Renderer *renderer) const {
    SDL_SetRenderDrawColor(renderer, 0xff, 0x0, 0x0, 0xff);
    int _x = x * block_size + (block_size - red_dot_size) / 2;
    int _y = y * block_size + (block_size - red_dot_size) / 2;
    SDL_Rect rect{_x, _y, red_dot_size, red_dot_size};
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

void draw_map(SDL_Renderer *renderer, std::set<Edge *> edges, std::vector<Point *> points, 
              Point *lhs, Point *rhs, std::vector<Point *> *ways) {
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
    if (lhs && rhs) {
        lhs->drawRedDot(renderer);
        rhs->drawRedDot(renderer);
        // draw the ways ... 
        for (int i = 1; i < ways->size(); ++i) {
            int x1, x2, y1, y2;
            (*ways)[i]->center(&x1, &y1);
            (*ways)[i - 1]->center(&x2, &y2);
            
            SDL_SetRenderDrawColor(renderer, 0xff, 0x0, 0x0, 0xff);
            SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
        }
    }
    SDL_RenderPresent(renderer);
}

void print_matrix(const std::vector<std::unordered_map<int, double>> &ohd, int edges) {
  int i, j;
  std::cout << "\n";
  std::cout << "  Distance matrix:\n";
  std::cout << "\n";
  std::cout << "\t";
  for (i = 0; i < edges; i++)
    std::cout << "\t[" << i << "]";
  std::cout << std::endl;
  for (i = 0; i < edges; i++) {
    std::cout << "\t[" << i << "]";
    for (j = 0; j < edges; j++) {
      if (!ohd[i].count(j)) {
        std::cout << "\tInf";
      } else {
        std::cout << "\t" << ohd[i].at(j);
      }
    }
    std::cout << "\n";
  }
}
static double get_val(const std::vector<std::unordered_map<int, double>> &ohd, int i, int j) {
  if (!ohd[i].count(j)) return i4_huge;
  else return ohd[i].at(j);
}

// return the way between lhs and rhs
std::vector<int> dijkstra(Point *lhs, Point *rhs, const std::vector<std::unordered_map<int, double>> &ohd) {
    int nv = ohd.size();
    std::cout << "dijkstra from Point " << lhs->getId() << " and " << rhs->getId() << std::endl;
    std::vector<bool> connected(nv, false);
    std::vector<int> point_to(nv, -1);
    double *dist_to;
    int i;
    int step;
    int mv; // the index of the nearest unconnected node.
    double md; // the distance from node 0 to the nearest unconnected node.
    
    connected[lhs->getId()] = true;
    dist_to = new double[nv];
  
    // init dist_to to one_step distance;
    for (i = 0; i < nv; ++i)
        dist_to[i] = get_val(ohd, lhs->getId(), i);
  
    // every step will find a new node connect to the tree.
    for (step = 1; step < nv; ++step) {
        md = i4_huge;
        mv = -1;
        for (i = 0; i < nv; ++i) {
            if (!connected[i] && dist_to[i] < md) {
                md = dist_to[i];
                mv = i;
            }
        }
  
        // can not find any vertex
        if (mv == -1) {
            std::cout << "\n";
            std::cout << "DIJKSTRA_DISTANCE - Warning!\n";
            std::cout << "  Search terminated early.\n";
            std::cout << "  Graph might not be connected.\n";
            break;
        }
  
        // find a good vertex, so mark it connected.
        connected[mv] = true;
  
        // update the dist_to (relax the finded vertex)
        for (i = 0; i < nv; ++i) {
            // the connected can be the best
            if (!connected[i]) {
                if (dist_to[i] > dist_to[mv] + get_val(ohd, mv, i)) {
                    dist_to[i] = dist_to[mv] + get_val(ohd, mv, i);
                    point_to[i] = mv; 
                }
            }
        }
    }

    delete [] dist_to;
    return point_to;
}

int main() {
    int edges;
    std::cout << "How many edges you want to generate? " << std::endl;
    std::cin >> edges;

    // The dijkstra map 
    std::vector<std::unordered_map<int, double>> ohd(edges * edges);

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
    Point *lhs = nullptr, *rhs = nullptr;

    // Kruskal's Algorithm
    std::set<Bag *> bags;
    std::vector<Point *> points;
    std::vector<std::vector<Point *>> 
        points_map(edges, std::vector<Point *>(edges));
    std::vector<Edge *> edges_vec;

    int current_id = 0;
    for (int y = 0; y < edges; ++y) {
        for (int x = 0; x < edges; ++x) {
            Point *point = new Point(x, y, current_id++);
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
    draw_map(renderer, edges_to_draw, points, lhs, rhs, nullptr);
    
    // 
    // shuffer the edge array
    //
    std::shuffle(edges_vec.begin(), edges_vec.end(), std::default_random_engine());
    auto it = edges_vec.begin();

    SDL_Event ev;
    bool running = true;

    // 
    // in the event loop, 
    // should init the dijkstra map  
    // 
    while (bags.size() != 1 && running) {
        // handle event (QUIT)
        while (SDL_PollEvent(&ev) != 0) {
            if (ev.type == SDL_QUIT) {
                running = false;
            }
        }

        std::pair<Point *, Point *> p 
                = (*it)->getPoints();
        if (p.first->parent() == p.second->parent()) {
            // they are in the same bag!!!
            // do nothing 
            ;
        } else {
            Bag *lhs = p.first->parent();
            Bag *rhs = p.second->parent();

            // connect this two point. 
            // the distance is 1.0
            ohd[p.first->getId()][p.second->getId()] = 1.0;
            ohd[p.second->getId()][p.first->getId()] = 1.0;

            // add rhs to lhs
            lhs->merge(rhs);

            // remove rhs
            bags.erase(rhs);
            delete rhs;

            edges_to_draw.erase(*it);
        }
        ++it;

        // SDL_Delay(1);

        draw_map(renderer, edges_to_draw, points, lhs, rhs, nullptr);
    }

    // print the matrix
#ifdef PRINT_MARTIX
    print_matrix(ohd, edges * edges);
#endif

    // for (Edge *edge : edges_vec) {
    //     auto points = edge->getPoints();
    //     std::cout << *points.first << ", " << *points.second << std::endl;
    // }

    // 
    // click sensor!!
    // 
    running = true;

    std::vector<Point *> points_;
    while (running) {
        draw_map(renderer, edges_to_draw, points, lhs, rhs, &points_);
        // handle event (QUIT)
        while (SDL_PollEvent(&ev) != 0) {
            if (ev.type == SDL_QUIT) {
                running = false;
            } else if (ev.type == SDL_MOUSEBUTTONDOWN) {
                int mouseX = ev.motion.x;
                int mouseY = ev.motion.y;
                
                int modX = mouseX / block_size;
                int modY = mouseY / block_size;

                Point *p = points_map[modX][modY];
                std::cout << "mouse down! mouse.x = " 
                          << mouseX << ", mouse.y = " << mouseY 
                          << ", [" << modX << ", " << modY << "]" 
                          << ", point.id = " << p->getId() << std::endl;
                

                if (!lhs) {
                    lhs = p;
                } else if (!rhs) {
                    rhs = p;

                    std::vector<int> point_to = dijkstra(lhs, rhs, ohd);
                    points_.clear();
                    points_.push_back(rhs);

                    std::cout << std::endl;
                    int i = rhs->getId();
                    for (;;) {
                        int id = point_to[i];
                        std::cout << "point to " << i << " is " << id << std::endl;
                        if (id == -1 || id == lhs->getId()) {
                            break;
                        } else {
                            i = id;
                        }
                        points_.push_back(points[i]);
                    }
                    points_.push_back(lhs);
                } else {
                    lhs = p;
                    rhs = nullptr;
                }
            }
        }

        SDL_Delay(100);
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

    // 
    // Destory window and renderer
    //
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);

    // quit SDL 
    SDL_Quit();

}
