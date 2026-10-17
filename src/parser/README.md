# parser - ROS 2 драйвер LC-AH127C (Bewis, RS485)

## Установка
    sudo apt install libserial-dev python3-serial ros-jazzy-sensor-msgs
    cd ~/apparat && colcon build --packages-select parser && source install/setup.bash

## Диагностика датчика (без ROS)
    sudo chmod 666 /dev/ttyUSB0
    python3 src/parser/scripts/ah127c_diag.py /dev/ttyUSB0
Скрипт перебирает скорости, шлёт запросы, пишет лог ah127c_diag_*.log.

## Запуск с датчиком
    ros2 launch parser parser.launch.py port:=/dev/ttyUSB0 baud:=9600 rate_hz:=10
Параметры: port, baud (2400..115200), rate_hz (0,5,10,20,25,50), axis_map ("x,y,z", "y,x,-z" ...),
save_settings (true - записать настройки в датчик), debug (печать сырых байт).
Ограничение: 57 байт * 10 бит * rate_hz <= baud. При 9600 - не выше 10 Гц. Для 50 Гц:
    python3 scripts/ah127c_diag.py /dev/ttyUSB0 --set-baud 115200
    ros2 launch parser parser.launch.py baud:=115200 rate_hz:=50

Топики: imu_full_data (Float32MultiArray, 16 значений: roll,pitch,yaw град; ax,ay,az м/с²;
gx,gy,gz град/с; mx,my,mz Гаусс; qx,qy,qz,qw), imu/data (sensor_msgs/Imu), imu/mag.

## Тест без датчика
    socat -d -d PTY,link=/tmp/ttyEmulator,raw,echo=0 PTY,link=/tmp/ttyROS,raw,echo=0
    python3 src/parser/scripts/emulator.py --rate 10 --noise
    ros2 launch parser parser.launch.py port:=/tmp/ttyROS
