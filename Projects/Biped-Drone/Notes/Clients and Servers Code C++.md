---
date: 2025-05-08
course: Robotic
topic: ROS2 humble
engagement: "2"
tags:
  - projects
  - biped-drone
  - notes
---
**Relationships**: [[ROS2 Humble]] [[Action Servers and Clients]]  [[C++ Practice 1]] [[Projects/Biped-Drone/Notes/Modern C++|Modern C++]]

### ✅ **1. SERVER CODE BREAKDOWN: `FibonacciActionServer`**

---

#### **Includes & Setup**

```cpp
#include <functional>
#include <memory>
#include <thread>
#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "action_tutorials_cpp/visibility_control.h"
```

- Brings in ROS2 and C++ standard libraries.
    
- Includes the action definition (`fibonacci.action`) and macros for component registration.

---

#### **Namespace and Class Declaration**

```cpp
namespace action_tutorials_cpp
{
class FibonacciActionServer : public rclcpp::Node
```

- Defines the node inside a namespace.
    
- Inherits from `rclcpp::Node`, making it a ROS2 node.

---

#### **Type Aliases**

```cpp
using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;
```

- Shorthand for action type and goal handle.

---

#### **Constructor**

```cpp
explicit FibonacciActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("fibonacci_action_server", options)
```

- Initializes node with name `"fibonacci_action_server"`.

```cpp
this->action_server_ = rclcpp_action::create_server<Fibonacci>(
      this, "fibonacci",
      std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
      std::bind(&FibonacciActionServer::handle_cancel, this, _1),
      std::bind(&FibonacciActionServer::handle_accepted, this, _1));
```

- Creates an action server that handles goals, cancels, and acceptance logic.

---

#### **Goal Handler**

```cpp
rclcpp_action::GoalResponse handle_goal(...)
```

- Logs the incoming goal.
    
- Accepts and immediately starts execution.

---

#### **Cancel Handler**

```cpp
rclcpp_action::CancelResponse handle_cancel(...)
```

- Accepts cancellation request without validation.

---

#### **Goal Acceptance Handler**

```cpp
void handle_accepted(...)
```

- Detaches a thread to run `execute()` asynchronously, keeping the executor non-blocked.

---

#### **Execution Logic**

```cpp
void execute(...)
```

- Retrieves goal.
    
- Initializes Fibonacci sequence.
    
- Runs a loop based on `goal->order`, computing and publishing feedback.
    
- Cancels or completes goal depending on `rclcpp::ok()` or `is_canceling()`.


---

#### **Node Registration**

```cpp
RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionServer)
```

- Enables dynamic loading of the component.

---

### ✅ **2. CLIENT CODE BREAKDOWN: `FibonacciActionClient`**

---

#### **Includes & Setup**

```cpp
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
```

- Same essential includes as the server.


---

#### **Namespace and Class Declaration**

```cpp
namespace action_tutorials_cpp
{
class FibonacciActionClient : public rclcpp::Node
```

- Node for sending goals to the action server.

---

#### **Type Aliases**

```cpp
using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;
```

---

#### **Constructor**

```cpp
explicit FibonacciActionClient(const rclcpp::NodeOptions & options)
  : Node("fibonacci_action_client", options)
```

- Initializes node.
    
- Creates action client and a timer to delay goal sending.

```cpp
this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");
this->timer_ = this->create_wall_timer(...)
```

- Uses wall timer to avoid sending goal before the server is ready.


---

#### **send_goal()**

```cpp
if (!this->client_ptr_->wait_for_action_server()) {
  RCLCPP_ERROR(...);
  rclcpp::shutdown();
}
```

- Waits for server availability.


```cpp
auto goal_msg = Fibonacci::Goal();
goal_msg.order = 10;
```

- Sets the Fibonacci order.

```cpp
auto send_goal_options = ...
```

- Defines callbacks for goal response, feedback, and result.

---

#### **Callbacks**

- **`goal_response_callback()`**: Confirms acceptance or rejection.
    
- **`feedback_callback()`**: Logs partial sequence from feedback.
    
- **`result_callback()`**: Handles result or logs if aborted/canceled.

---

#### **Node Registration**

```cpp
RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionClient)
```

---

### 🔧 **3. POSSIBLE PARAMETER CONFIGURATIONS**

You can add ROS parameters via launch files or dynamically:

|Parameter Name|Type|Description|
|---|---|---|
|`goal_order`|int|Determines the number of terms to compute in the Fibonacci sequence.|
|`feedback_rate`|double|Sleep rate in Hz, used in the loop.|
|`cancel_on_threshold`|bool|Automatically cancels if sequence exceeds a limit (custom logic needed).|
|`action_name`|string|Custom action server name instead of `"fibonacci"`.|
|`auto_shutdown_client`|bool|If `true`, shuts down client node after result received.|

**Implementation**: use `this->declare_parameter<T>(...)` in constructor and `get_parameter(...)` before use.

---

### 🚀 **4. CHALLENGE TASK**

**Implement a parameterized Fibonacci action system**:

- Add a `goal_order` parameter (default 10) to the client.
    
- Add a `feedback_rate` parameter (Hz) to the server's loop rate.
    
- Add a launch file to start both client and server with customized parameters.

> **Bonus**: Modify the server to publish a progress percentage with each feedback.

Would you like help setting up the parameter logic or the launch file?

___
# Structure
Great question — **yes, action servers and clients generally follow a similar structure in ROS 2**, but with important **roles and responsibilities** that define their differences. Here's how their structure compares:

---

### ✅ **Shared Core Components**

Both **action servers** and **clients** in ROS 2 use the same underlying concepts:

|Component|Server Role|Client Role|
|---|---|---|
|`Goal`|Receives and evaluates it|Sends the goal|
|`Feedback`|Publishes progress info|Receives and logs/shows progress|
|`Result`|Returns final outcome|Waits for and handles result|
|`GoalHandle`|Manages goal state (accepted, canceled, etc.)|Reference to a sent goal|
|`create_server` / `create_client`|Sets up action handling infrastructure|Sets up connection to server|

---

### ⚙️ **Server-Side Structure**

Always includes:

1. `handle_goal()` – Accept/reject goal.
    
2. `handle_cancel()` – Accept/reject cancel requests.
    
3. `handle_accepted()` – Launches a thread for `execute()`.
    
4. `execute()` – Runs action logic, sends feedback and result.
    

---

### 🚀 **Client-Side Structure**

Always includes:

1. `wait_for_action_server()` – Waits for server availability.
    
2. `send_goal()` – Sends goal with:
    
    - `goal_response_callback`
        
    - `feedback_callback`
        
    - `result_callback`
        

---

### 🔁 **Pattern Consistency**

This consistent pattern allows developers to **easily switch between different actions** (e.g., `FollowPath`, `DockRobot`, etc.) with minimal structural changes — only the action-specific message types and logic differ.

---

### 🧩 **Customization**

You can still:

- Add parameters.
    
- Use timers, threads, or services.
    
- Inject action logic via plugins or components.
    

---

Would you like a diagram or template that generalizes the structure for quick reuse?