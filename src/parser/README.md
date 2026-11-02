
##Для теста без датчика
1) Создать два виртуальных порта: socat -d -d PTY,link=/tmp/ttyEmulator,raw,echo=0 PTY,link=/tmp/ttyROS,raw,echo=0
2) Запустить эмулятор из папки со скриптом: python3 emulator.py
3) Запустить ноду: ros2 run parser imu_node

##Запуск с реальным датчиком (перед этим поменять порт в коде)
1) Дать права порту: sudo chmod 666 /dev/ttyUSB0
2) Запустить ноду: ros2 run parser imu_node
