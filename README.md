# 1294C Chromatic Library from Push Back

Simple Motion Control Library for VEX V5 based off PROS 4.

To get started, run the following commands to set up your PROS environment
```
pros make clean
pros build-compile-commands
```

## Key Files 
This includes both the header .hpp in include/ and the source .cpp in src/.
**main** -- no way
**subsystems** -- allows you control subsystems. use the header file's Body enum to name the states, and modify set_body and update_body in the .cpp
**lidar** -- use the empty side and fwd reset functions to begin writing distance resets. or just use your own interpretation
**config** -- all parameters and configuration things are here. very nice
**autons** -- all your autons are here. be careful that for the auton selector to work, you also need to update the competition initialize to match the autons you have as well as the autons.hpp enum and naming enum and functions.

## Feature List
We have a lot of features, and some of them have very cursed documentation of what their parameters do.
For the most part, function names should provide enough context to understand what it does, with extra notes being specified.

### Movements
- Swing to heading
- Face to point
- Turn to heading
- Drive distance
- Timed Kinematic Movement
- Move to Point
- Custom Move to Pose
- Slew Rate Limiting
- Explicit Lerp-based Motion Chaining

### Localization Options
- Encoder-based Odometry with IMU
- Distance Sensor Reset Support
- Field-Element Collision Raycasting for Distance Sensors

### Other Features
- Concurrent Subsystem Management and Control
- Basic Auton Selector
- Autonomous Movement Target Tracker
- Many shorthands for faster coding
