# ROS Navi Package

## :package: Packages Overview

- [`cannyone_bringup`](./cannyone_bringup): Launch files to bring up the hardware drivers (camera, imu, ...) for the real CannyOne robot
- [`cannyone_communication`](./cannyone_communication): Communication between Raspberry Pi and MCU, including publishing raw sensors data to it's related ROS topic
- [`cannyone_description`](./cannyone_description): URDF description of CannyOne including its sensors
- [`cannyone_simulation`](./cannyone_simulation): Simulation specific launch and configuration files for CannyOne

## CannyOne Bringup

### Docker Container Dependencies

- ros-noetic-cv-camera*
- ros-noetic-compressed-image-transport*
- ros-noetic-image-proc*


## CannyOne Description

### Docker Container Dependencies

- ros-noetic-urdf*
- ros-noetic-xacro


## CannyOne Simulation



## CannyOne Communication

### Raspberry Pi Dependencies

- Edit `/boot/firmware/config.txt`, then reboot the Raspberry Pi after adding the following line:

> dtoverlay=uart5  				 # Enable UART5


### Raspberry Pi UART Pins

| UART | TXD | RXD | CTS | RTS |   | Board| Pins | |
|------|-----|-----|-----|-----|---|------|------|-|
|uart0 |  14 |  15 |     |     |   |   8  |  10  |
|uart1 |  14 |  15 |     |     |   |   8  |  10  |
|uart2 |  0  |  1  |  2  |  3  |   |  27  |  28  |   (I2C)  |
|uart3 |  4  |  5  |  6  |  7  |   |   7  |  29  |
|uart4 |  8  |  9  |  10 |  11 |   |  24  |  21  |  (SPI0)  |
|uart5 |  12 |  13 |  14 |  15 |   |  32  |  33  |(gpio-fan)|


### Docker Container Dependencies

- pip install pyserial
- pip install pyyaml
- pip install asyncio



## CannyOne Camera


### Raspberry Pi Dependencies

- libraspberrypi-bin
- Edit `/boot/firmware/config.txt`, then reboot the Raspberry Pi after adding the following lines:

> start_x=1
> gpu_mem=128
> include syscfg.txt
> include usercfg.txt


### Docker Container Dependencies

- python3-pip
- pip install picamera
- pip install "picamera[array]"
- opencv-python
- libgl1-mesa-glx
- ros-noetic-camera*
- ros-noetic-usb-cam*
- Edit `/opt/ros/noetic/lib/camera_calibration/cameracalibrator.py`, then edit the following line:

> #!/usr/bin/env python ⟶ #!/usr/bin/env python 