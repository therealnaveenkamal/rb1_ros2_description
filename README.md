

# RB-1 ROS2 Controller

This project is a simple demonstration of Robotnik's RB-1 robot's controllers and various plugins. We will specifically explore the differential drive controller and elevator controller in this repo.

![RB-1 Robot](https://github.com/therealnaveenkamal/rb1_ros2_description/assets/80611084/7deeac87-7692-43be-908e-a12fc43ee2e2)


## Table of Contents

- [Introduction](#introduction)

- [Requirements](#requirements)

- [Installation](#installation)

- [Usage](#usage)

- [Configuration](#configuration)

- [Contributing](#contributing)

- [License](#license)

## Introduction

This project showcases a basic implementation of control of various joints through various custom hardware interfaces using their position, velocity, and effort parameters. 

## Requirements

- ROS (Robot Operating System)

- C++ Compiler

- Gazebo Simulation / Appropriate Robot Hardware

## Installation

1\. Clone the repository:

   ```
   git clone https://github.com/therealnaveenkamal/rb1_ros2_description.git
   ```

2\. Build the project:

   ```
   cd ~/ros2_ws
   colcon build && source install/setup.bash
   ```

## Usage

1\. Launch the RB-1 Controller:

   ```
   ros2 launch rb1_ros2_description rb1_ros2_xacro.launch.py
   ```

2\. The robot will take up to a few moments to load and start the controller managers. 

3\. You can monitor the robot's behavior in real-time.

4\. To check the proper functioning of the program, you may run the following commands:

  >     ros2 control list_controllers
	joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
	elevator_effort_controller[effort_controllers/JointGroupEffortController] active
	rb1_base_controller [diff_drive_controller/DiffDriveController] active

  >     ros2 control list_hardware_interfaces
	command interfaces
		        robot_elevator_platform_joint/effort [claimed]
		        robot_left_wheel_joint/velocity [claimed]
		        robot_right_wheel_joint/velocity [claimed]
	state interfaces
		         robot_elevator_platform_joint/effort
		         robot_elevator_platform_joint/position
		         robot_elevator_platform_joint/velocity
		         robot_left_wheel_joint/effort
		         robot_left_wheel_joint/position
		         robot_left_wheel_joint/velocity
		         robot_right_wheel_joint/effort
		         robot_right_wheel_joint/position
		         robot_right_wheel_joint/velocity
		         
		         
5\. Once the controllers and loaded and active, we can publish messages to the topic to play with the configured control managers. You have to run ```ros2 topic list``` or ```ros2 topic info /topic_name -v ``` to know the configured topics and their detailed configuration respectively.

 - Differential Drive Controller
	 To control the RB-1 robot's movement, execute the below command in a new terminal
	 
	```ros2 topic pub --rate 10 /rb1_base_controller/cmd_vel_unstamped geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0, z: 0.0}, angular: {x: 0.0,y: 0.0, z: 0.5}}"```

- Elevator Controller
	To control the elevator's z-axis movement, execute the below command where the minimum and maximum effort of the elevator platform is configured to be 0 and 10 respectively.

```ros2 topic pub /elevator_effort_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0]"```

## Configuration

You can configure the robot's controller parameters by modifying the .yaml file in `~/ros2_ws/src/rb1_ros2_description/config/rb1_controller.yaml`.

## Contributing

If you'd like to contribute to this project, please follow these guidelines:

1\. Fork the repository.

2\. Create a new branch for your feature: `git checkout -b feature-name`.

3\. Make and commit your changes: `git commit -m 'Add new feature'`.

4\. Push to the branch: `git push origin feature-name`.

5\. Create a pull request.

We welcome your contributions!

## License

This project was created under the supervision of the team at [The Construct](https://theconstructsim.com/)


## Disclaimer:

This package only modifies/adapts files from these repositories/packages:  
- [RobotnikAutomation/rb1_base_sim](https://github.com/RobotnikAutomation/rb1_base_sim) licensed under the BSD 2-Clause "Simplified" License
- [RobotnikAutomation/rb1_base_common/rb1_base_description](https://github.com/RobotnikAutomation/rb1_base_common/tree/melodic-devel/rb1_base_description), licensed under the BSD License
- [RobotnikAutomation/robotnik_sensors],(https://github.com/RobotnikAutomation/robotnik_sensors) licensed under the BSD License
