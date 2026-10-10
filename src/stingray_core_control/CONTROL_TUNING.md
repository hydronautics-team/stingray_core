# Separate surge and sway tuning

Enable the required axes through the existing loop flags. Bit 0 enables surge
and bit 1 enables sway:

```bash
ros2 topic pub --once /control/loop_flags std_msgs/msg/UInt8 '{data: 3}'
```

Publishing a setpoint selects the mode for that axis. Position setpoints are
absolute DVL coordinates in metres and remain active until replaced:

```bash
ros2 topic pub --once \
  /stingray_core_control_node/setpoint/surge/position \
  std_msgs/msg/Float64 '{data: 1.0}'

ros2 topic pub --once \
  /stingray_core_control_node/setpoint/sway/position \
  std_msgs/msg/Float64 '{data: 0.5}'
```

Velocity setpoints are in metres per second. They must be refreshed faster
than `command_timeout_sec`, otherwise the setpoint is changed to zero:

```bash
ros2 topic pub --rate 10 \
  /stingray_core_control_node/setpoint/surge/velocity \
  std_msgs/msg/Float64 '{data: 0.2}'

ros2 topic pub --rate 10 \
  /stingray_core_control_node/setpoint/sway/velocity \
  std_msgs/msg/Float64 '{data: 0.1}'
```

Each axis publishes an independent set of `std_msgs/msg/Float64` debug topics:

```text
/stingray_core_control_node/debug/<axis>/setpoint
/stingray_core_control_node/debug/<axis>/position
/stingray_core_control_node/debug/<axis>/velocity
/stingray_core_control_node/debug/<axis>/err_position
/stingray_core_control_node/debug/<axis>/output_pi
/stingray_core_control_node/debug/<axis>/feedback_speed
/stingray_core_control_node/debug/<axis>/measurement_rate
/stingray_core_control_node/debug/<axis>/out
```

`<axis>` is `surge`, `sway`, `heave`, `roll`, `pitch`, or `yaw`.

When `use_separate_horizontal_setpoints` is `true`, horizontal values from the
legacy `/control/data` topic are ignored while their axes are closed. This
prevents a joystick or bridge publisher from overriding the tuning topics.
Set the parameter to `false` to restore the compatibility behaviour where
`Twist.linear.x/y` select position mode and represent absolute DVL positions.
