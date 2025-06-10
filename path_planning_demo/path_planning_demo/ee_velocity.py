import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformListener
import numpy as np

class EEVelocityPublisher(Node):
    """
    A ROS 2 node that calculates and publishes the end-effector velocity
    based on the transformation from the 'world' frame to the 'tool0' frame.
    It computes the velocity as the change in position over time and publishes
    it as a Float64 message on the '/ee_velocity' topic.
    """
    def __init__(self):
        super().__init__('ee_velocity_publisher')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.prev_time = self.get_clock().now()
        self.prev_pos = None

        self.publisher = self.create_publisher(Float64, '/ee_velocity', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform('world', 'tool0', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            current_time = self.get_clock().now()
            dt = (current_time - self.prev_time).nanoseconds * 1e-9

            pos = trans.transform.translation
            current_pos = np.array([pos.x, pos.y, pos.z])

            if self.prev_pos is not None and dt > 0:
                velocity = (current_pos - self.prev_pos) / dt
                msg = Float64()
                msg.data = np.linalg.norm(velocity)
                self.publisher.publish(msg)

            self.prev_pos = current_pos
            self.prev_time = current_time

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {str(e)}")

def main():
    rclpy.init()
    node = EEVelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()