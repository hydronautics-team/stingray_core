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

## Development environment

The project provides a Dev Container with the required development tools and dependencies.

The recommended way to work with the project is through VS Code and the Dev Container.

### Start the development container

Open the repository in VS Code and use:

**Dev Containers: Reopen in Container**

The container provides the ROS 2 Humble development environment and the tools required to build and check the project.

### C++ development tools

The project uses:

- `clangd` — C++ language server
- `clang-format` — code formatting
- `clang-tidy` — static analysis

The repository contains a shared `.clang-format` configuration.

C++ files are automatically formatted on save in VS Code.

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
