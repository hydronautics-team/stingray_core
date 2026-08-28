# ROS 2 Topics

## Sensor interfaces

The canonical sensor topics are:

| Sensor | Topic |
|---|---|
| Depth | `/core/sensors/depth` |
| IMU | `/core/sensors/imu` |
| DVL | `/core/sensors/dvl` |

These topics are part of the public interface of the core system.

Driver-specific and raw communication topics are considered internal implementation details and must not be used by higher-level modules.

## Naming rules

- `/stingray_core/sensors/<sensor>` is used for raw sensor data.
- Sensor-specific processing should use separate topics.
- Localization outputs must not be published under `/stingray_core/sensors`.
- Topic names use lowercase `snake_case`.
