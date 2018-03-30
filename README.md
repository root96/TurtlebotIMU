# TurtlebotIMU

This is communiacion via UART between :
- NUCLEO-L476RG (publisher)
- Raspberry Pi 3 (listener)

Data is serializated using nanopb (Protocol Buffer for Embedded System). Nanopb is a small code-size Protocol Buffers implementation in ansi C. It is especially suitable for use in microcontrollers, but fits any memory restricted system.
The C-code written for Nucleo is just simple publisher. It is sending data via UART using DMA. And then in our Raspberry Pi
we can start ros-node written in python. To be honest this node is listener and publisher, because he is getting data via UART and sending it to rostopic /imu. Data from IMU can be visualizated using ROS-tools like for example rviz.

# Question
If you have any question, just write to me -> kamil.kielbasa13@gmail.com
