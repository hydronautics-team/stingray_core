# stingray_core

Part of [Stingray framework](https://github.com/hydronautics-team/stingray)

![stingray_core architecture](docs/arch.jpg)

## Dependencies

- [serial](https://github.com/jinmenglei/serial.git) - for communication with stm32 and etc.

## Repository structure

After refactoring, packages are grouped in `src/` as follows:

- `src/stingray_core_communication/`
  - `stingray_core_communication`
  - `asio_cmake_module`
  - `io_context`
  - `serial`
  - `serial_driver`
  - `udp_driver`
  - `Hydrolib-soft`
- `src/stingray_core_sensors/`
  - `battery_sensor`
  - `pressure_sensor`
  - `ms5837_pressure_sensor`
  - `dvl-a50`
  - `dvl_msgs`
  - `vectornav`
- `src/stingray_core_devices/`
  - `servo_device`
  - `power_control`
- Left as top-level packages:
  - `src/stingray_core_control`
  - `src/stingray_core_launch`


## Docker

In ~/stingray_core/

**run**

```bash
./docker/run.sh
```

**rebuild**

```bash
./docker/build.sh
```

## Run ROV

In ~/stingray_core/

```bash
./run_rov.sh
```
