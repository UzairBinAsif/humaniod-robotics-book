---
sidebar_position: 2
title: ROS 2 Communication Patterns
---

# ROS 2 Communication Patterns

In the world of robotics, different parts of the system need to communicate constantly. A sensor needs to send data to a processing unit, a planner needs to send commands to a motor controller, and a user interface needs to display the robot's current status. ROS 2, the Robotic Nervous System, provides a robust set of communication patterns to handle this complex web of information exchange.

The three fundamental patterns are Nodes, Topics, and Services.

## Nodes: The Brain Cells

A **Node** is the smallest unit of computation in a ROS 2 system. Think of it as a single, focused program responsible for one specific task. For example, you might have:

-   A `/camera_driver` node responsible for capturing images from a camera.
-   A `/image_processor` node that detects objects in those images.
-   A `/motor_controller` node that moves the robot's joints.

Each node is an independent executable that can be started, stopped, and restarted without affecting the others, making the overall system highly modular and resilient.

## Topics: The Public Broadcast System

**Topics** are the primary way that nodes communicate in a many-to-many fashion. They work on a publish-subscribe model.

-   A node **publishes** messages (data packets) to a specific topic.
-   Any number of other nodes can **subscribe** to that topic to receive those messages.

This is like a public radio broadcast. The sender (publisher) doesn't know or care who is listening. It just sends the data out. The listeners (subscribers) don't know who sent the data; they just tune into the channel they care about.

This decoupling is powerful. You can easily add a new `/logger` node that subscribes to sensor topics to record data, without having to change the original sensor driver node at all.

### Code Example: A Python 'Talker' Node

Let's create a simple "Talker" node that simulates publishing the position of a humanoid's shoulder joint. This node will continuously publish a message containing a floating-point number to a topic named `/shoulder_joint_position`.

```python title="talker_node.py"
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

class JointPositionPublisher(Node):
    """
    A ROS 2 node that simulates publishing the position of a humanoid shoulder joint.
    The position oscillates back and forth to simulate movement.
    """
    def __init__(self):
        super().__init__('shoulder_joint_publisher')
        self.publisher_ = self.create_publisher(Float64, 'shoulder_joint_position', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Float64()
        # Simulate an oscillating joint movement between -1.57 and 1.57 radians (90 degrees)
        msg.data = 1.57 * math.sin(self.i * 0.1)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    joint_position_publisher = JointPositionPublisher()
    try:
        rclpy.spin(joint_position_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        joint_position_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

In this example:
1.  We create a `JointPositionPublisher` class that inherits from `rclpy.node.Node`.
2.  In the constructor, we create a publisher for messages of type `Float64` on the `shoulder_joint_position` topic.
3.  A timer calls the `timer_callback` function every 0.1 seconds.
4.  The callback calculates a new joint position using a sine wave and publishes it in a `Float64` message.

## Services: The Request-Response Handshake

While Topics are great for continuous data streams, sometimes you need a direct request-response interaction. This is where **Services** come in.

A Service has two parts:

-   A **Service Server** (provided by one node) that performs a specific task when called and returns a result.
-   A **Service Client** (used by another node) that calls the server, sends a request, and waits for a response.

This is a one-to-one communication, like making a function call to another program. It's perfect for tasks that have a clear beginning and end, such as:

-   Requesting a robot to move to a specific named pose (e.g., "home position").
-   Asking a perception system to identify an object in a specific region of an image.
-   Triggering a calibration routine.

By combining Nodes, Topics, and Services, you can build complex, distributed robotics applications from simple, reusable components.
