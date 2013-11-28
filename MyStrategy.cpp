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
#include <stack>
#include <memory>

using namespace model;
using namespace std;

const int MAX_SHOOTING_RANGE = 10;

struct Point;
struct Mission;

typedef vector<vector<float>> WeightMap;
typedef list<Point> Path;
typedef vector<Trooper> Troopers;

WeightMap empty_map;
WeightMap safety_map;
WeightMap explore_map;
set<Point> obstacles;
map<TrooperType, stack<unique_ptr<Mission>>> missions;
map<TrooperType, Path> paths;

void fill_empty_map(const World&, WeightMap& target);
void init_safety_map(const World&);
void init_explore_map(const World&);

void update_explore_map(const World&);

pair<float, Point> get_explore_point(const World&);

bool astar(const World&, const Point&, const Point&,
        const set<Point>&, const WeightMap&, Path&);

void simple_move(const Point&, const Point&, Move&);

void validate_missions();
bool is_mission_active(const Trooper&);
void assign_mission(const Trooper&, unique_ptr<Mission>);
void evaluate_mission(const World&, const Trooper&, Move& move);

void draw_map(const World&, const WeightMap&, const Troopers&, const Trooper&);

struct Point {
    int x;
    int y;

    Point(): x(0), y(0) {}

    Point(int x, int y): x(x), y(y) {}

    Point(const Point& o): x(o.x), y(o.y) {}

    bool operator==(const Point& o) const {
        return x == o.x && y == o.y;
    }

    bool operator!=(const Point& o) const {
        return x != o.x or y != o.y;
    }

    bool operator<(const Point& o) const {
        return x < o.x || (x == o.x && y < o.y);
    }
};

struct Mission {
    Mission() {}

    Point target;

    bool canceled;

    virtual void eval(const World&, const Trooper&, Move&) = 0;
};

struct MissionExplore: Mission {
    MissionExplore() {}

    void eval(const World& world, const Trooper& self, Move& move) {
        auto start = Point(self.getX(), self.getY());
        Path path;
        astar(world, start, target, obstacles, empty_map, path);
        printf("[MISSION] Explore: %d, %d\n", target.x, target.y);
        simple_move(Point(self.getX(), self.getY()), path.front(), move);
        paths[self.getType()] = path;
    }
};

MyStrategy::MyStrategy() {}

void MyStrategy::move(const Trooper& self, const World& world,
        const Game& game, Move& move) {
    if (empty_map.empty()) {
        fill_empty_map(world, empty_map);
    }

    if (safety_map.empty()) {
        init_safety_map(world);
    }

    if (explore_map.empty()) {
        fill_empty_map(world, explore_map);
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

    validate_missions();

    if (!is_mission_active(self)) {
        Point interest = get_explore_point(world).second;
        auto mission = unique_ptr<Mission>(new MissionExplore);
        mission->target = interest;
        assign_mission(self, std::move(mission));
    }

    evaluate_mission(world, self, move);

    draw_map(world, safety_map, world.getTroopers(), self);
}

void assign_mission(const Trooper& self, unique_ptr<Mission> mission) {
    missions[self.getType()].push(std::move(mission));
}

void evaluate_mission(const World& world, const Trooper& self, Move& move) {
    missions.at(self.getType()).top()->eval(world, self, move);
}

bool overlay_troopers(const Point& point, const Troopers& troopers) {
    for (auto it = troopers.begin(); it != troopers.end(); it++) {
        auto& trooper = *it;

        if (trooper.getX() == point.x && trooper.getY() == point.y) {
            printf("\e[1;37m");
            printf("@");
            return true;
        }
    }

    for (auto it = troopers.begin(); it != troopers.end(); it++) {
        auto& trooper = *it;

        if (paths.count(trooper.getType()) > 0){
            auto& path = paths[trooper.getType()];
            auto it = find(path.begin(), path.end(), point);
            if (it != path.end()) {
                printf("\e[37m");
                printf("x");
                return true;
            }
        }
    }

    return false;
}

void draw_map(const World& world, const WeightMap& map_data,
        const Troopers& troopers, const Trooper& self) {
    printf("\e[34m");
    printf("%4s", " ");
    for (int x = 0; x < world.getWidth(); x++) {
        printf("%3d ", x);
    }
    printf("\n");
    printf("%4s", "   +");
    for (int x = 0; x < world.getWidth(); x++) {
        printf("----");
    }
    printf("\n");
    for (int y = 0; y < world.getHeight(); y++) {
        printf("\e[0;34m");
        printf("%3d|", y);
        for (int x = 0; x < world.getWidth(); x++) {
            printf("\e[0m");
            auto point = Point(x, y);

            if (world.getCells().at(x).at(y) != FREE) {
                printf("████");
            } else {
                printf("%3.0f", map_data.at(x).at(y));

                bool printed = false;
                printed = printed || overlay_troopers(point, troopers);

                if (!printed) {
                    printf(" ");
                }
            }
        }
        printf("\n");
    }
    printf("\n");
}

bool is_mission_active(const Trooper& self) {
    if (missions.count(self.getType()) == 0) {
        return false;
    }

    if (missions.at(self.getType()).empty()) {
        return false;
    } else {
        return true;
    }
}

void validate_missions() {
    for (auto it = missions.begin(); it != missions.end(); it++) {
        auto& missions_stack = it->second;
        while (!missions_stack.empty() && missions_stack.top()->canceled) {
            missions_stack.pop();
        }
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

void neighbors(const Point pivot, const World& world, list<Point>& result) {
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
        safety_map.push_back(vector<float>());
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
                        (to_visible ?   1 : 0) -
                        (from_visible ? 1 : 0);
                }
            }
        }
    }

    printf("TIME: %f\n", double(clock() - start_clock) / CLOCKS_PER_SEC);
}

void fill_empty_map(const World& world, WeightMap& target) {
    target.reserve(world.getWidth());
    for (int x = 0; x < world.getWidth(); x++) {
        target.push_back(vector<float>());
        target.at(x).reserve(world.getHeight());
        for (int y = 0; y < world.getHeight(); y++) {
            target.at(x).push_back(0);
            target.at(x).at(y) = 0;
        }
    }
}

void update_explore_map(const World& world) {
    for (int x = 0; x < world.getWidth(); x++) {
        for (int y = 0; y < world.getHeight(); y++) {
            for (vector<Trooper>::const_iterator it = world.getTroopers().begin();
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
    pair<float, Point> maximum = make_pair(0, Point(0, 0));

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
        printf("Moving EAST\n");
        move.setDirection(EAST);
    }

    if (pivot.x - 1 == point.x) {
        printf("Moving WEST\n");
        move.setDirection(WEST);
    }

    if (pivot.y + 1 == point.y) {
        printf("Moving SOUTH\n");
        move.setDirection(SOUTH);
    }

    if (pivot.y - 1 == point.y) {
        printf("Moving NORTH\n");
        move.setDirection(NORTH);
    }
}

bool astar(const World& world,
        const Point& start, const Point& goal,
        const set<Point>& obstacles,
        const WeightMap& costs,
        Path& path) {

    set<Point> closed_set;
    priority_queue<pair<float, Point>> open_queue;
    set<Point> open_set;
    std::map<Point, Point> came_from;

    open_set.insert(start);
    open_queue.push(make_pair(0, start));

    std::map<Point, float> g_score;
    std::map<Point, float> f_score;
    g_score.insert(make_pair(start, 0));
    f_score.insert(make_pair(start, -manhatten(start, goal)));

    while (!open_queue.empty()) {
        Point current = open_queue.top().second;

        if (current == goal) {
            while (current != start) {
                path.push_back(current);
                current = came_from.at(current);
            }

            reverse(path.begin(), path.end());
            return true;
        }

        open_queue.pop();
        closed_set.insert(current);

        list<Point> neighbors_list;
        neighbors(current, world, neighbors_list);

        for (list<Point>::iterator it = neighbors_list.begin();
                it != neighbors_list.end(); ++it) {
            Point neighbor = *it;

            if (obstacles.count(neighbor) > 0) {
                continue;
            }

            float expected_g_score = g_score.at(current) +
                costs.at(current.x).at(current.y) +
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

