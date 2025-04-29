---
date: 2025-04-28
course: Robotic
topic: ROS2 humble
engagement: "2"
tags:
  - projects
  - biped-drone
  - notes
---
ROS2 is a node communication protocol used to interchange data. It consists of a set of nodes and its message interchange protocols.
*Node*: Each one is set for an unique purpose. It can send/receive data through topics, services, parameters, and actions. A single executable can contain several nodes.
- *Topic*: Bus for message interchange. It can be `one-to-one`,`many-to-many`, `one-to-many`o `many-to-one`.
	- *Publisher*: Node that sends a message.
	- *Subscriber*: Node that receives a message.
	- *Properties*: Type (`-t`), data (`echo`), info s/p (`info`), interface show (`interface show`), frequency (`hz`), bandwidth (`bw`), publish (`pub`).
- *Service*: Based on `call-and-response`. They only provide data when called by a client.
	- *Service types*: Request, response.
	- *Properties*: Similar to *topics*, but `pub` $\leftrightarrow$ `call`.
- *Parameters*: Node setup.
	- *Properties*: get (`get`), set (`set`), parameter list per node (`dump`), load parameters from `.yaml` file (`load`).
- *Actions*: Long-running procedure call with feedback. It can be cancelled or preempted.
	- *Action client*: Who sets the goal.
	- *Action server*: Who executes towards the goal.

To be able to work with ROS2 Humble, it is necessary to start a workspace in the repository where the code will be included so that the packages are available to be used from terminal (path must be appended to bash).
- *ROS tutorial workspace path*: `~/Documents/Academics/Projects/Biped-Drone/Notes`
	- *Print Environmental Variables*: `printenv | grep -i ros`
- *To build workspace*: `colcon build`
	- *Useful args.*: 
		- `--packages-up-to` builds the package you want, plus all its dependencies, but not the whole workspace.
		- `--symlink-install` saves you from having to rebuild every time you tweak python scripts.
		- `--event-handlers console_direct+` shows console output while building (can otherwise be found in the `log` directory).
		- `--executor sequential` processes the packages one by one instead of using parallelism.
- *Setup an Overlay*: Source `install/local_setup.bash` in the root of the workspace. (from a new terminal).
- *Libraries*: ROS uses the `rclpy`library for python and the `rclpp`library for C++.
	- *Launch*: To launch several nodes at a time use `ros2 launch <filename>`