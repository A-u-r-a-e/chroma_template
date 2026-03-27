#pragma once
#include "api.h"
#include "chromatic.hpp"

// we keep the function with the enum because it helps with debugging
extern enum autons {LEFT=0, RIGHT=1, SOLO=3, SKILLS=4, CIRCLE=5, TEST=6} auton_select;
inline const char* get_auton_name(autons auton) {
    switch (auton) {
    case LEFT: return "Left";
    case RIGHT: return "Right";
    case SOLO: return "Solo";
    case SKILLS: return "Skills";
    case CIRCLE: return "60in Square";
    case TEST: return "Testing";
    default: return "Unknown";
    }
}

enum struct CompState{REST, AUTON, OPCONTROL, DISABLE};
extern std::atomic<CompState> comp_state;

extern const ms auton_pollrate;
extern const ms op_pollrate;

extern pros::Controller master;

extern pros::MotorGroup left_mg;
extern pros::MotorGroup right_mg;
extern pros::IMU inertial;

extern chromatic::Differential drivebase;
extern std::unique_ptr<chromatic::Odometry> localizer;

extern chromatic::FieldWalls walls;
extern std::vector<chromatic::FieldElementBox> elements;

extern chromatic::PID drive_pid;
extern chromatic::PID heading_pid;
extern chromatic::PID turn_pid;
extern chromatic::PID swing_pid;
extern chromatic::MotionController pilot;
