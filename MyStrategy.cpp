#include "MyStrategy.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <cstdio>
#include <set>
#include <map>
#include <utility>
#include <queue>
#include <list>
#include <algorithm>

using namespace model;
using namespace std;

const int MAX_SHOOTING_RANGE = 10;

struct Point {
    int x;
    int y;

    Point(int x, int y): x(x), y(y) {} 

    Point(const Point& o): x(o.x), y(o.y) {}

    bool operator==(const Point& o) const {
        return x == o.x && y == o.y;
    }

    bool operator<(const Point& o) const {
        return x < o.x || (x == o.x && y < o.y);
    }
};

vector< vector< float > > safety_map;
vector< vector< float > > explore_map;
set< Point > obstacles;

MyStrategy::MyStrategy() {}

void init_safety_map(const World&);
void init_explore_map(const World&);
void update_explore_map(const World&);
pair< float, Point > get_explore_point(const World&);
bool astar(const World&, const Point, const Point,
        const set< Point >&, list< Point >&);

void MyStrategy::move(const Trooper& self, const World& world,
        const Game& game, Move& move) {
    if (safety_map.empty()) {
        init_safety_map(world);
    }

    if (explore_map.empty()) {
        init_explore_map(world);
    }

    update_explore_map(world);

    if (obstacles.empty()) {
        for (int x = 0; x < world.getWidth(); x++) {
            for (int y = 0; y < world.getHeight(); y++) {
                if (world.getCells().at(x).at(y) != FREE) {
                    obstacles.insert(Point(x, y));
                }
            }
        }
    }

    Point interest = get_explore_point(world).second;
    list< Point > path;
    astar(world, interest, Point(self.getX(), self.getY()), obstacles, path);

    printf("\n");
    for (int x = 0; x < world.getWidth(); x++) {
        for (int y = 0; y < world.getHeight(); y++) {
            if (world.getCells().at(x).at(y) != FREE) {
                printf(" TTT ");
            } else {
                list< Point >::iterator it = find(path.begin(), path.end(),
                    Point(x, y));
                if (it != path.end()) {
                    printf("  X  ");
                } else {
                    printf("%4.1f ", explore_map.at(x).at(y));
                }
            }
        }
        printf("\n");
    }
}

float manhatten(const Point& start, const Point& end) {
    return abs(start.x - end.x) + abs(start.y - end.y);
}

float euclid(const int x1, const int y1, const int x2, const int y2) {
    return hypot(x1 - x2, y1 - y2);
}

float euclid(const Point& start, const Point& end) {
    return euclid(start.x, start.y, end.x, end.y);
}

void neighbors(const Point pivot, const World& world, list< Point >& result) {
    if (pivot.x > 0) {
        result.push_back(Point(pivot.x - 1, pivot.y));
    }

    if (pivot.y > 0) {
        result.push_back(Point(pivot.x, pivot.y - 1));
    }

    if (pivot.x < world.getWidth() - 1) {
        result.push_back(Point(pivot.x + 1, pivot.y));
    }

    if (pivot.y < world.getHeight() - 1) {
        result.push_back(Point(pivot.x, pivot.y + 1));
    }
}

void init_safety_map(const World& world) {
    clock_t start_clock = clock();
    safety_map.reserve(world.getWidth());
    for (int x = 0; x < world.getWidth(); x++) {
        safety_map.push_back(vector< float >());
        safety_map.at(x).reserve(world.getHeight());
        for (int y = 0; y < world.getHeight(); y++) {
            safety_map.at(x).push_back(0);

            for (int x2 = 0; x2 < world.getWidth(); x2++) {
                for (int y2 = 0; y2 < world.getHeight(); y2++) {
                    if (hypot(x - x2, y - y2) > MAX_SHOOTING_RANGE) {
                        continue;
                    }

                    bool from_visible = world.isVisible(
                        MAX_SHOOTING_RANGE,
                        x, y, STANDING,
                        x2, y2, STANDING);

                    bool to_visible = world.isVisible(
                        MAX_SHOOTING_RANGE,
                        x2, y2, STANDING,
                        x, y, STANDING);

                    safety_map.at(x).at(y) +=
                        (from_visible ? 1 : 0) -
                        (to_visible ?   1 : 0);
                }
            }
        }
    }

    printf("TIME: %f\n", double(clock() - start_clock) / CLOCKS_PER_SEC);
} 

void init_explore_map(const World& world) {
    explore_map.reserve(world.getWidth());
    for (int x = 0; x < world.getWidth(); x++) {
        explore_map.push_back(vector< float >());
        explore_map.at(x).reserve(world.getHeight());
        for (int y = 0; y < world.getHeight(); y++) {
            explore_map.at(x).push_back(0);
            explore_map.at(x).at(y) = 0;
        }
    }
}

void update_explore_map(const World& world) {
    for (int x = 0; x < world.getWidth(); x++) {
        for (int y = 0; y < world.getHeight(); y++) {
            for (vector< Trooper >::const_iterator it = world.getTroopers().begin();
                    it != world.getTroopers().end(); ++it) {
                Trooper trooper = *it;
                if (world.isVisible(
                        trooper.getVisionRange(),
                        trooper.getX(), trooper.getY(), STANDING,
                        x, y, STANDING)) {
                    explore_map.at(x).at(y) = 0;
                } else {
                    explore_map.at(x).at(y) += euclid(x, y,
                        trooper.getX(), trooper.getY()) / 100.0;
                }
            }
        }
    }
}

pair< float, Point> get_explore_point(const World& world) {
    pair< float, Point > maximum = make_pair(0, Point(0, 0));

    for (int x = 0; x < world.getWidth(); x++) {
        for (int y = 0; y < world.getHeight(); y++) {
            if (maximum.first <= explore_map.at(x).at(y)) {
                maximum.first = explore_map.at(x).at(y);
                maximum.second = Point(x, y);
            }
        }
    }

    return maximum;
}

void simple_move(const Point& pivot, const Point& point, Move& move) {
    move.setAction(MOVE);
    //  N
    // W E
    //  S
    if (pivot.x + 1 == point.x) {
        move.setDirection(EAST);
    }

    if (pivot.x - 1 == point.x) {
        move.setDirection(WEST);
    }

    if (pivot.y + 1 == point.y) {
        move.setDirection(NORTH);
    }

    if (pivot.y - 1 == point.y) {
        move.setDirection(SOUTH);
    }
}

bool astar(const World& world,
        const Point start, const Point goal,
        const set< Point >& obstacles,
        list< Point >& path) {

    set< Point > closed_set;
    priority_queue< pair< float, Point > > open_queue;
    set< Point > open_set;
    std::map< Point, Point > came_from;

    open_set.insert(start);
    open_queue.push(make_pair(0, start));

    std::map< Point, float > g_score;
    std::map< Point, float > f_score;
    g_score.insert(make_pair(start, 0));
    f_score.insert(make_pair(start, -manhatten(start, goal)));

    while (!open_queue.empty()) {
        Point current = open_queue.top().second;

        if (current == goal) {
            while (came_from.count(current) > 0) {
                path.push_back(current);
                current = came_from.at(current);
            }

            reverse(path.begin(), path.end());
            return true;
        }

        open_queue.pop();
        closed_set.insert(current);

        list< Point > neighbors_list;
        neighbors(current, world, neighbors_list);

        for (list< Point >::iterator it = neighbors_list.begin();
                it != neighbors_list.end(); ++it) {
            Point neighbor = *it;

            if (obstacles.count(neighbor) > 0) {
                continue;
            }

            float expected_g_score = g_score.at(current) +
                euclid(current, neighbor);
            float expected_f_score = expected_g_score +
                manhatten(neighbor, goal);
            
            if (closed_set.count(neighbor) > 0) {
                continue;
            }

            if (open_set.count(neighbor) == 0) {
                g_score.insert(make_pair(neighbor, expected_g_score));
                f_score.insert(make_pair(neighbor, expected_f_score));
                came_from.insert(make_pair(neighbor, current));
                open_set.insert(neighbor);
                open_queue.push(make_pair(-expected_f_score, neighbor));
            } else {
                if (expected_f_score < f_score.at(neighbor)) {
                    came_from.at(neighbor) = current;
                }
            }
        }
    }

    return false;
}

