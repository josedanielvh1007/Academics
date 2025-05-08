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
**Relationships:** [[Projects/Biped-Drone/Notes/Modern C++|Modern C++]] [[Projects/Biped-Drone/Notes/Modern C++ Practice|Modern C++ Practice]]
# ROS 2 Humble
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
---
## `rosdep`
- Command line utility for identifying and installing the dependencies to build or install a package. Information is saved within the `package.xml`file. 
- Meta-package manager to find the appropriate packages to install. Often invoked within a workspace to install the dependencies needed to work within it.
	- It can work either within a single package or a directory of packages.
	- All the dependencies should be specified in the `package.xml`file (manually-made). 
	- `<depend>`: Dependencies that should be provided both at build time and run time.
	- `<build_depend>`: Dependencies that should only be used at build time.
	- `<exec_depend>`: Dependencies that should only be used at run time.
	- `<test_depend>`: Dependencies needed only for tests.
	- `<build_export_depend>`: Dependencies used in root programs both for building and executing.
- **What to Include?**:
	- -If the package you want to depend in your package is ROS-based, AND has been released into the ROS ecosystem [1](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html#id2), e.g. `nav2_bt_navigator`, you may simply use the name of the package. You can find a list of all released ROS packages in [https://github.com/ros/rosdistro](https://github.com/ros/rosdistro) at `<distro>/distribution.yaml` (e.g. `humble/distribution.yaml`) for your given ROS distribution.
	- If you want to depend on a non-ROS package, something often called “system dependencies”, you will need to find the keys for a particular library. In general, there are two files of interest:
	    - [rosdep/base.yaml](https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml) contains the `apt` system dependencies
	    - [rosdep/python.yaml](https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml) contains the Python dependencies
- *To use*: `rosdep install --from-paths src -y --ignore-src`
---
## Actions
*Within the Workspace*: `ros2 pkg create <nameofpackage>`
- Actions must be created within the `actions`directory of the package and are `.action`.
- They are defined in the form
	```action
	# Request
	---
	# Result
	--- 
	# Feedback
	```
	- `requests` messages initialize new goals.
	- `result`messages is sent from an action server to client when the action is done.
	- `feedback`periodical messages from action server to client updates about a goal.
- Add the action to the `CMakeLists.cmake`file:
	```cmake
	find_package(rosidl_default_generators REQUIRED)
	
	rosidl_generate_interfaces(${PROJECT_NAME}
	  "action/<nameoftheaction>.action"
	) 
	```
- Add the required dependencies to the `package.xml`file:
	```xml
	<buildtool_depend>rosidl_default_generators</buildtool_depend>
	
	<depend>action_msgs</depend>
	
	<member_of_group>rosidl_interface_packages</member_of_group>
	```
- Source the workspace: `source ./install/setup.bash`
- Check the install: `ros2 interface show <pathtoaction>`
### Writing action servers and clients
In the `src`folder of the workspace: `ros2 pkg create --dependencies action_tutorials_interfaces rclcpp rclcpp_action rclcpp_components -- action_tutorials_cpp`
Action servers require 6 things:
1. The templated action name: `<actiontemplatedname>`.
2. A ROS 2 node to add the action to: `this`.
3. The action name: `<nameoftheaction>`.
4. A callback function to handle goals: `handle_goal`.
5. A callback function to handle cancellation: `handle_cancel`.
6. A callback function to handle goal accepts: `handle_accept`.
