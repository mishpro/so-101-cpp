# so-101-cpp
Manipulation tasks @ SO101 Robot Arm

`send` can calculate joint angles for a Cartesian target and send them to the
servos:

```sh
./send --cartesian X Y Z
```

Coordinates are specified in meters in the URDF base frame. The URDF path,
end-effector frame, and IK parameters are configured in `send_config.json`.

For keyboard control from the neutral pose:

```sh
./send --cartesian-control
```

Use `T/G` for X, `D/A` for Y, `W/S` for Z, and `Q` to quit. The movement
step is configured by `cartesian_step`.
