# hardware_interface
Arduino code used to establish two-way communication between ROS and Arduino. Arduino reads encoder data and publishes /tf and /odom topics. It also receives /cmd_vel data and uses it to achieve steady locomotion using PID.
