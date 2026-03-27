#include "config.hpp"

enum autons auton_select{};
std::atomic<CompState> comp_state{CompState::REST};

const ms auton_pollrate = 10;
const ms op_pollrate = 10;

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({1, 2, -3}, pros::MotorGearset::blue, pros::MotorEncoderUnits::counts);
pros::MotorGroup right_mg({-4, -5, 6}, pros::MotorGearset::blue, pros::MotorEncoderUnits::counts);
pros::IMU inertial(7);

using namespace chromatic;

Differential drivebase(left_mg, right_mg, 3.25, (24.0/23.5)*(48.0/72.0), (12), 0.5);
std::unique_ptr<Odometry> localizer(new EncodersIMU(drivebase, inertial));

FieldWalls walls{70.25, 70.25, 70.25, 70.25};
std::vector<FieldElementBox> elements = {
    FieldElementBox{-24, 24, -24, 24}
};

using SC = SettleCondition;
using SR = SlewRate;

PID fwd_pid{
    7.0, 0.0, 1.0, 15.0,
    SC(1.0, 100), SC(2.0, 500),
    30, 50, 50
};
PID head_pid{
    5.0, 0.0, 3.5, PI / 6.0,
    SC(to_rad(1), 100), SC(to_rad(2), 500),
    2*PI, 2.5*PI, 2.5*PI
};
PID turn_pid{
    10.0, 3.0, 1.5, PI / 6.0,
    SC(to_rad(1), 100), SC(to_rad(2), 500),
    2*PI, 2.5*PI, 2.5*PI
};
PID swing_pid{
    80, 16, 8, to_rad(30.0),
    SC(to_rad(1), 100), SC(to_rad(2), 500),
    30, 120, 120
};
MotionController pilot{
    drivebase, localizer,
    fwd_pid, turn_pid, head_pid, swing_pid,
    SR{40}, SR{PI}, SR{PI}, SR{80}
};
