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
        self.get_logger().info('Shoulder Joint Publisher has been started.')

    def timer_callback(self):
        msg = Float64()
        # Simulate an oscillating joint movement between -1.57 and 1.57 radians (90 degrees)
        msg.data = 1.57 * math.sin(self.i * 0.1)
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing joint position: "%s"' % msg.data)
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
