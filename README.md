# so-101-cpp
Manipulation tasks @ SO101 Robot Arm

## Cartesian control

Build and run the keyboard Cartesian controller:

```sh
cmake -S . -B build
cmake --build build --target cartesian_control
./build/cartesian_control
```

The controller reads `send_config.json`, loads the configured URDF with
Pinocchio, and uses `gripper_frame_link` as the Cartesian target. It reads the
current servo positions before starting, so the initial target matches the
physical robot.

Keyboard controls: `T/G` move along X, `A/D` along Y, `W/S` along Z, and `Q`
exits. Each key press moves the target by `cartesian_step_m` and sends the
resulting five arm joint angles through the configured serial port. The
gripper joint is held at its current position.
