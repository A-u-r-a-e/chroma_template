#include "main.h"

using namespace chromatic;

void initialize() {
	pros::lcd::initialize();

	// hardware initialization
	drivebase.set_brake_type(COAST);
	comp_state = CompState::REST;

	// calibration
    localizer->calibrate();

    // tasks
    pros::Task odom_task([&]{localizer->localize(10);});
    pros::Task body_task([&]{run_body(10);});

	master.rumble(".");
}

void disabled() {}

void competition_initialize() {
    auto switch_auto = [&](){
        auton_select = static_cast<autons>((auton_select + 1) % 7);
        master.print(0, 0, "Auton: %s", get_auton_name(auton_select));
    };


    master.clear();
    // auton selection
    while (comp_state == CompState::REST) {

        master.print(0, 0, "Auton: %s", get_auton_name(auton_select));
        pros::lcd::print(5, "Auton: %s", get_auton_name(auton_select));
        if (master.get_digital_new_press(BY)) switch_auto();
        delay_for(50);
    }

    master.rumble("-");
}

void autonomous() {
    comp_state = CompState::AUTON;

    drivebase.set_brake_type(BRAKE);

    pilot.refresh_cache();
    pilot.set_pollrate(auton_pollrate);

    switch (auton_select) {
        case LEFT: left_auto(); break;
        case RIGHT: right_auto(); break;
        case SOLO: solo_auto(); break;
        case SKILLS: skills_auto(); break;
        case CIRCLE: circle_auto(); break;
        case TEST: test_auto(); break;
        default: break;
    }

    comp_state = CompState::REST;
}

void opcontrol() {

    // autonomous();

    pilot.interrupt();

    master.rumble(".");
    comp_state = CompState::OPCONTROL;
    drivebase.set_brake_type(COAST);

	while (true) {

		int fwd = master.get_analog(LY);
		int turn = master.get_analog(RX);

		if (abs(fwd)+abs(turn) != 0) {
		    pilot.override_arcade(fwd, turn);
		} else {
		    pilot.override_brake();
		}

		delay_for(op_pollrate);
	}

	comp_state = CompState::DISABLE;
	localizer->stop_loop();
	delay_for(50);
}
