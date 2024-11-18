# ME495 Embedded Systems Homework 3 (Part I)

Author: David Khachatryan

This package simulates a differential drive robot in Gazebo.

## Quickstart
1. Use `ros2 launch diff_drive ddrive.launch.xml` to start the arena and turtle simulation.
2. Here is a video of the differential drive robot navigating a gazebo world
3. The messages are sent from a ros node `flip` that publishes to `cmd_vel` with `1 / 0.4` hz frequency.
4. Here is a demo:

<video src="https://github.com/user-attachments/assets/80343329-1a53-41b0-b000-4ca64625e59c" controls="controls" style="max-width: 100%; height: auto;">
    Your browser does not support the video tag.
</video>
