#include "lidar.hpp"

using namespace chromatic;

void update_lidar(ms pollrate) {
    while (comp_state != CompState::DISABLE) {
        Pose pose = localizer->get_pose();

        // update lidars
        // DoubleTOF or SingleTOF

        delay_for(pollrate);
    }
}

// checks to see if a pose is currently aiming at an object, true if true
bool check_collision(Pose pose) {
    for (auto& object : elements) {
        if (object.projection_collision(pose)) return true;
    }
    return false;
}

bool side_reset(Quad quadrant) {

}

bool fwd_reset(Quad quadrant, bool calculate_heading) {

}
