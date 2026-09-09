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
physical robot. The target frame is configured by `end_effector` in
`send_config.json`, so another URDF frame can be selected without changing the
source code.

IK tuning is configured in the same file: `ik_damping` controls numerical
stability, `ik_max_iterations` limits iterations per key press,
`ik_position_tolerance_m` is the Cartesian stopping tolerance, and
`ik_max_joint_step_rad` limits one joint update.

Keyboard controls: `T/G` move along X, `A/D` along Y, `W/S` along Z, and `Q`
exits. Each key press moves the target by `cartesian_step_m` and sends the
resulting five arm joint angles through the configured serial port. The
gripper joint is held at its current position.

{
  "port": "/dev/ttyACM0",
  "speed": 600,
  "joint_speed_divisor": 1.2,
  "urdf_path": "~/develop/urdf/so101_new_calib.urdf",
  "cartesian_step_m": 0.01, // размер перемещения целевой точки при нажатии клавиши
  "end_effector": "gripper_frame_link", // frame, относительно которого считается Cartesian-положение
  "ik_damping": 0.0001, // 1e-5 ... 1e-2 меньшее значение — выше точность около нормальных поз, но хуже устойчивость около сингулярностей; большее значение — устойчивее расчёт, но движение менее точное и более «мягкое».
  "ik_max_iterations": 100, // число итераций IK на одно нажатие клавиши
  "ik_position_tolerance_m": 0.005, // критерий остановки, например ошибка менее 1 мм
  "ik_max_joint_step_rad": 0.1 // ограничение изменения одного сустава за одну итерацию. Защищает от резких движений
}