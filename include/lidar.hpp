#pragma once
#include "api.h"
#include "chromatic.hpp"
#include "config.hpp"
#include "subsystems.hpp"

enum struct Quad{Q1 = 1, Q2 = 2, Q3 = 3, Q4 = 4};

void update_lidar(ms pollrate = 50);

bool check_collision(chromatic::Pose pos);
