+++
title = "Koch Robot Teleop Setup: Combining Embedded Systems, Control Theory, and ROS2"
author = ["Zheng Qu"]
date = 2026-01-08T00:00:00+01:00
lastmod = 2026-03-12T20:25:06+01:00
tags = ["ROS2", "Embedded-Systems", "Control-Theory", "Teleoperation", "ESP32", "Koch-Robot"]
draft = false
weight = 2001
+++

Fresh out of the New Year break, I finally got around to reviving a pair of small [Koch robots](https://github.com/jess-moss/koch-v1-1) that I 3D-printed back in 2024. They had been collecting dust after some early experiments with an ACT model, so I decided to give them a second life — this time as a small claw-machine-style teleop setup that my son can eventually play with.


## Technical Highlights {#technical-highlights}

-   Teleoperation via a joystick connected to an **ESP32** board from the ROSCon 2025 ros2_control workshop
-   **picoRoS** running directly on the ESP32, communicating with the robot over **Zenoh**
-   Handwritten kinematics and Jacobian based on screw theory
-   Nullspace velocity control for redundancy handling
-   Thoughtful joystick mapping for intuitive teleop
-   Controllers implemented using **ros2_control**

A fun way to combine embedded systems, control theory, ROS2, and some practical motivation at home.


## Demo {#demo}

Check out the video below where I teleop the follower arm with a joystick:

{{< figure src="https://raw.githubusercontent.com/Maverobot/koch_ros/main/demo.gif" caption="Jogging the follower arm with joystick input" >}}


## Code {#code}

All code is open source and available on GitHub:

-   [koch_ros](https://github.com/Maverobot/koch_ros) — where a leader-follower controller is also implemented (the follower arm can mirror the leader arm's movements)
-   [picoros_playground](https://github.com/Maverobot/picoros_playground)
-   [dynamixel_hardware](https://github.com/Maverobot/dynamixel_hardware)
