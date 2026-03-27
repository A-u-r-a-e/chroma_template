#pragma once
#include "api.h"
#include "chromatic.hpp"
#include "config.hpp"

enum struct Body{NOTHING};
enum struct Pneumatic{RETRACTED, EXTENDED};

void set_body(bool off = false);

void update_body();

void run_body(ms pollrate = 10);
