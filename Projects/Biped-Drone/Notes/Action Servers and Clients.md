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
**Relationships:** [[ROS2 Humble]]
# Actions
To create an action, it is recommended to start an action-dedicated folder within a package inside the workspace to ensure proper organization of the nodes.
Actions are defined in `.action` files in the form:
```
# Request
---
# Result
---
# Feedback
```
Where a _request_ is sent by a client to a server initiating a new goal, the _result_ is sent from the server to the client when the goal is done, and the _feedback_ is sent in the same direction from the latter with updates about a goal.
## Building an action
Before using the action, it is necessary to pass the definition to the `rosidl` generation pipeline. `example with "action/Fibonacci.action"` In the `CMakeLists.txt`file:
```
find_package(rosidl_default_generators= REQUIRED)

rosidl_generate_interfaces($(PROJECT_NAME)
	"action/Fibonacci.action"
)
```
And in the `package.xml` file:
```
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<depend>action_msgs</depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```
>_Note: Remember to build in the parent folder after configured._

After everything has been properly set up, source the workspace with `. install/setup.bash`
To check the action definition: `ros2 interface show <path_to_the_action>` (if it exists, it'll print the action in the terminal).
# Writing an Action Server and Client
Actions are a form of asynchronous communication in ROS. Clients send goal requests to the servers, and the servers send goal feedback and results to action clients. The first step to create an action server/client is to define a new package and setup the dependencies to the other packages (**important!**).
## 1. Visibility control
To make the package work it is necessary to add a `visibility_control.h` file. Adapted from [https://gcc.gnu.org/wiki/Visibility]():
```h
#ifndef ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_
#define ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ACTION_TUTORIALS_CPP_EXPORT __attribute__ ((dllexport))
    #define ACTION_TUTORIALS_CPP_IMPORT __attribute__ ((dllimport))
  #else
    #define ACTION_TUTORIALS_CPP_EXPORT __declspec(dllexport)
    #define ACTION_TUTORIALS_CPP_IMPORT __declspec(dllimport)
  #endif
  #ifdef ACTION_TUTORIALS_CPP_BUILDING_DLL
    #define ACTION_TUTORIALS_CPP_PUBLIC ACTION_TUTORIALS_CPP_EXPORT
  #else
    #define ACTION_TUTORIALS_CPP_PUBLIC ACTION_TUTORIALS_CPP_IMPORT
  #endif
  #define ACTION_TUTORIALS_CPP_PUBLIC_TYPE ACTION_TUTORIALS_CPP_PUBLIC
  #define ACTION_TUTORIALS_CPP_LOCAL
#else
  #define ACTION_TUTORIALS_CPP_EXPORT __attribute__ ((visibility("default")))
  #define ACTION_TUTORIALS_CPP_IMPORT
  #if __GNUC__ >= 4
    #define ACTION_TUTORIALS_CPP_PUBLIC __attribute__ ((visibility("default")))
    #define ACTION_TUTORIALS_CPP_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ACTION_TUTORIALS_CPP_PUBLIC
    #define ACTION_TUTORIALS_CPP_LOCAL
  #endif
  #define ACTION_TUTORIALS_CPP_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ACTION_TUTORIALS_CPP__VISIBILITY_CONTROL_H_
```
## 2. Writing an action server
Code snippets can be written either in python or in C++
### 2.1 Action server in C++
>_Example: [Writing the action server code - ROS humble documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html#writing-the-action-server-code)_

First, it is necessary to include in the program the code dependencies:
```cpp
#include <functional>
#include <memory>
#include <thread>

#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "action_tutorials_cpp/visibility_control.h"
namespace action_tutorials_cpp   // Package name
{
```
Next, create a class derived of `rclpp::Node` :
```cpp
class FibonacciActionServer : public rclcpp::Node
```
And initialize the node from the class:
```cpp
  explicit FibonacciActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("fibonacci_action_server", options)
  ```
## Prerequisites
To create an action server/client it is necessary to have already created the [[#Actions]]. 
