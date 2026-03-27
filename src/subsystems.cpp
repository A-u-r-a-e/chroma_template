#include "subsystems.hpp"

using namespace chromatic;

void set_body(bool off) {

}

void update_body() {

}

void run_body(ms pollrate) {
    while (comp_state != CompState::DISABLE) {

        if (comp_state == CompState::REST) {
            set_body(true);
        } else {
            update_body();
        }

		delay_for(pollrate);
    }
}
