* **Serial Driver**

A package which which encapsulates basic receiving and sending of serial data.

Provided within this package is the following executabe:
- serial_bridge: combined both receiver and sender nodes into one

Provided within this package also is a `serial_driver` library without the ROS2 dependencies which could be used elsewhere.

`serial_bridge` parameters:
- `device_name`, `baud_rate`, `flow_control`, `parity`, `stop_bits`
- `direction_gpio` (`int`, default `-1`): optional TX/RX direction GPIO for RS485-like control.
  - `-1`: disabled
  - `>= 0`: GPIO number in Linux sysfs global numbering (`/sys/class/gpio/gpioX`)
  - behavior: sets GPIO to `0` before TX, returns to `1` after TX completion, and forces `1` during open/close
