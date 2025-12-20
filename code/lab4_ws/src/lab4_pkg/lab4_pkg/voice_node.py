import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class VoiceToActionNode(Node):
    """
    A mock ROS 2 node that simulates a voice-to-action pipeline.
    It listens for a simulated voice command and publishes a navigation goal.
    """
    def __init__(self):
        super().__init__('voice_to_action_node')
        self.goal_publisher_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.timer = self.create_timer(10.0, self.simulate_voice_command)
        self.get_logger().info('Voice-to-Action node has been started.')

    def simulate_voice_command(self):
        """
        This function simulates receiving a voice command like
        "go to the red cube". In a real system, this would be triggered by
        a speech-to-text engine like Whisper.
        """
        self.get_logger().info('Simulated voice command: "Go to the charging station"')

        # This would be the output of a VLM and perception system
        target_x = 2.5
        target_y = -1.0

        # Publish the navigation goal
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = target_x
        goal_msg.pose.position.y = target_y
        goal_msg.pose.orientation.w = 1.0  # Face forward

        self.goal_publisher_.publish(goal_msg)
        self.get_logger().info(f'Published navigation goal to x={target_x}, y={target_y}')

def main(args=None):
    rclpy.init(args=args)
    voice_node = VoiceToActionNode()
    try:
        rclpy.spin(voice_node)
    except KeyboardInterrupt:
        pass
    finally:
        voice_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
