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
#include <cstdarg>

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
WeightMap team_map;

set<Point> obstacles;

map<TrooperType, stack<unique_ptr<Mission>>> missions;

map<TrooperType, Path> paths;

map<Point, Trooper> enemies;

void fill_empty_map(const World&, WeightMap& target);
void init_safety_map(const World&);
void init_explore_map(const World&);

void update_explore_map(const World&);
void update_team_map(const World&);
void update_enemies(const World&, const Trooper&);

pair<float, Point> get_explore_point(const World&);

bool astar(const World&, const Point&, const Point&,
        const WeightMap&, Path&, ...);
Point bfs(const World&,
        const Point&, int,
        std::function<bool(Point&)> cond_func, ...);

void simple_move(const Point&, const Point&, Move&);

void validate_missions();
bool is_mission_active(const Trooper&);
void assign_mission(const Trooper&, unique_ptr<Mission>);
void evaluate_mission(const Game&, const World&, const Trooper&, Move&);

void draw_map(const World&, const WeightMap&, const Troopers&, const Trooper&);


int get_max_walk_len(const Game&, const Trooper&);

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

    virtual void eval(const Game& game, const World&, const Trooper&,
            Move&) = 0;
};

struct MissionExplore: Mission {
    MissionExplore(const World& world) {
        target = get_explore_point(world).second;
    }

    void eval(const Game& game, const World& world, const Trooper& self,
            Move& move) {
        WeightMap local_safety_map;
        fill_empty_map(world, local_safety_map);
        auto start = Point(self.getX(), self.getY());
        Path path;
        astar(world, start, target, team_map, path, &obstacles, nullptr);
        int offset = 0;
        int max_walk_len = get_max_walk_len(game, self);
        printf("1\n");
        for_each(path.begin(), path.end(),
            [&] (Point& point) {
                offset += 1;
                if (max_walk_len <= offset) {
                    return;
                }

                printf("2\n");
                auto minimum = make_pair(0.0, point);
                bfs(world, point, max_walk_len - offset,
                    [&] (Point& p) -> bool {
                        printf("3 %d %d\n", p.x, p.y);
                        if (safety_map[p.x][p.y] < minimum.first) {
                            minimum.first = safety_map[p.x][p.y];
                            minimum.second = p;
                        }

                        return false;
                    },
                    &obstacles, nullptr);

                printf("BFS %d %d\n", minimum.second.x, minimum.second.y);
            });
        printf("[MISSION] Explore: %d, %d\n", target.x, target.y);
        simple_move(start, path.front(), move);
        paths[self.getType()] = path;
    }
};

MyStrategy::MyStrategy() {}

void MyStrategy::move(const Trooper& self, const World& world,
        const Game& game, Move& move) {
    clock_t start_clock = clock();

    if (empty_map.empty()) {
        fill_empty_map(world, empty_map);
    }

    if (safety_map.empty()) {
        init_safety_map(world);
    }

    if (explore_map.empty()) {
        fill_empty_map(world, explore_map);
    }

    if (team_map.empty()) {
        fill_empty_map(world, team_map);
    }

    update_explore_map(world);
    update_team_map(world);
    update_enemies(world, self);

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
        auto mission = unique_ptr<Mission>(new MissionExplore(world));
        assign_mission(self, std::move(mission));
    }

    evaluate_mission(game, world, self, move);

    printf("TIME: %.4f\n", double(clock() - start_clock) / CLOCKS_PER_SEC);

    draw_map(world, team_map, world.getTroopers(), self);
}

void assign_mission(const Trooper& self, unique_ptr<Mission> mission) {
    missions[self.getType()].push(std::move(mission));
}

void evaluate_mission(const Game& game, const World& world,
        const Trooper& self, Move& move) {
    missions.at(self.getType()).top()->eval(game, world, self, move);
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
                float to_print = map_data[x][y];
                if (to_print < 0) {
                    printf("\e[31m");
                    to_print *= -1;
                } else {
                    printf("\e[0m");
                }

                if (to_print >= 10) {
                    printf("%3.0f", to_print);
                } else {
                    printf("%3.1f", to_print);
                }

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
            for (auto it = world.getTroopers().begin();
                    it != world.getTroopers().end(); it++) {
                Trooper trooper = *it;
                if (world.isVisible(
                        trooper.getVisionRange(),
                        trooper.getX(), trooper.getY(), STANDING,
                        x, y, STANDING)) {
                    explore_map[x][y] = 0;
                } else {
                    explore_map[x][y] += euclid(x, y,
                        trooper.getX(), trooper.getY()) / 100.0;
                }
            }
        }
    }
}

void update_team_map(const World& world) {
    for (int x = 0; x < world.getWidth(); x++) {
        for (int y = 0; y < world.getHeight(); y++) {
            team_map[x][y] = 0;
            for_each(world.getTroopers().begin(), world.getTroopers().end(),
                [&] (Trooper trooper) {
                    float distance = euclid(x, y, trooper.getX(), trooper.getY());
                    team_map[x][y] += distance / 10.0;
                });
        }
    }
}

void update_enemies(const World& world, const Trooper& self) {
    for_each(world.getTroopers().begin(), world.getTroopers().end(),
        [&] (Trooper trooper) {
            if (trooper.getPlayerId() == self.getPlayerId()) {
                return;
            }

            auto old_enemy = find_if(enemies.begin(), enemies.end(),
                [&] (pair<const Point, Trooper> p) -> bool {
                    return p.second.getId() == self.getId();
                });
            if (old_enemy != enemies.end()) {
                enemies.erase(old_enemy);
            }

            enemies[Point(trooper.getX(), trooper.getY())] = trooper;
    });
}

pair<float, Point> get_explore_point(const World& world) {
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
        printf("[SIMPLE MOVE] Moving EAST >\n");
        move.setDirection(EAST);
    }

    if (pivot.x - 1 == point.x) {
        printf("[SIMPLE MOVE] Moving WEST <\n");
        move.setDirection(WEST);
    }

    if (pivot.y + 1 == point.y) {
        printf("[SIMPLE MOVE] Moving SOUTH ^\n");
        move.setDirection(SOUTH);
    }

    if (pivot.y - 1 == point.y) {
        printf("[SIMPLE MOVE] Moving NORTH v\n");
        move.setDirection(NORTH);
    }
}

int get_step_cost(const Game& game, const Trooper& self) {
    switch (self.getStance()) {
        case STANDING:
            return game.getStandingMoveCost();
        case KNEELING:
            return game.getKneelingMoveCost();
        case PRONE:
            return game.getProneMoveCost();
        default:
            return -1;
    }
}

int get_max_walk_len(const Game& game, const Trooper& self) {
    return self.getActionPoints() / get_step_cost(game, self);
}

bool astar(const World& world,
        const Point& start, const Point& goal,
        const WeightMap& costs,
        Path& path, ...) {

    set<Point> closed_set;
    priority_queue<pair<float, Point>> open_queue;
    set<Point> open_set;
    std::map<Point, Point> came_from;

    va_list obstacles_args;
    va_start(obstacles_args, path);
    while (true) {
        auto obstacles_arg = va_arg(obstacles_args, const set<Point>*);
        if (obstacles_arg == nullptr) {
            break;
        }

        closed_set.insert(obstacles_arg->begin(), obstacles_arg->end());
    }

    va_end(obstacles_args);

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

        for_each(neighbors_list.begin(), neighbors_list.end(),
            [&] (Point& neighbor) {
                if (closed_set.count(neighbor) > 0) {
                    return;
                }

                float expected_g_score = g_score.at(current) +
                    costs.at(current.x).at(current.y) +
                    euclid(current, neighbor);
                float expected_f_score = expected_g_score +
                    manhatten(neighbor, goal);

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
            });
    }

    return false;
}

Point bfs(const World& world,
        const Point& start, int max_depth,
        std::function<bool(Point&)> cond_func, ...) {

    set<Point> closed_set;
    set<Point> open_set;
    queue<pair<int, Point>> open_queue;

    va_list obstacles_args;
    va_start(obstacles_args, cond_func);
    while (true) {
        auto obstacles_arg = va_arg(obstacles_args, const set<Point>*);
        if (obstacles_arg == nullptr) {
            break;
        }

        closed_set.insert(obstacles_arg->begin(), obstacles_arg->end());
    }

    va_end(obstacles_args);

    open_set.insert(start);
    open_queue.push(make_pair(0, start));

    while (!open_queue.empty()) {
        auto current_pair = open_queue.front();
        auto current_depth = current_pair.first;
        auto current = current_pair.second;

        open_queue.pop();

        list<Point> neighbors_list;
        neighbors(current, world, neighbors_list);

        if (current_depth >= max_depth) {
            continue;
        }

        if (cond_func(current)) {
            return current;
        }

        closed_set.insert(current);

        for_each(neighbors_list.begin(), neighbors_list.end(),
            [&] (Point& neighbor) {
                printf("N %d %d\n", neighbor.x, neighbor.y);
                if (closed_set.count(neighbor) > 0) {
                    return;
                }

                if (open_set.count(neighbor) > 0) {
                    return;
                }

                open_queue.push(make_pair(current_depth + 1, neighbor));
            });
    }

    return start;
}
