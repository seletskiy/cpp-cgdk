#include "MyStrategy.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <cstdio>

using namespace model;
using namespace std;

const int MAX_SHOOTING_RANGE = 10;

vector< vector< int > > safety_map;

MyStrategy::MyStrategy() {}

void MyStrategy::move(const Trooper& self, const World& world, const Game& game, Move& move) {
    if (safety_map.empty()) {
        clock_t start_clock = clock();
        safety_map.reserve(world.getWidth());
        for (int x = 0; x < world.getWidth(); x++) {
            safety_map.push_back(vector< int >());
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

    printf("\n");
    for (int x = 0; x < world.getWidth(); x++) {
        for (int y = 0; y < world.getHeight(); y++) {
            if (world.getCells()[x][y] != FREE) {
                printf("OOO ");
            } else {
                printf("%3d ", safety_map.at(x).at(y));
            }
        }
        printf("\n");
    }
}
