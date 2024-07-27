import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations

class StaticFramePublisher(Node):

    def __init__(self):
        super().__init__('static_frame_publisher')
        self._tf_broadcaster = StaticTransformBroadcaster(self)

        # Publish the static transform once at startup
        self.publish_static_transforms()

    def publish_static_transforms(self):
        tf_msg = TransformStamped()

        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'base_link'
        tf_msg.child_frame_id = 'lidar'

        tf_msg.transform.translation.x = 0.2  # Example values, change as needed
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.3

        quat = tf_transformations.quaternion_from_euler(0, 0, 0)  # No rotation in this example
        tf_msg.transform.rotation.x = quat[0]
        tf_msg.transform.rotation.y = quat[1]
        tf_msg.transform.rotation.z = quat[2]
        tf_msg.transform.rotation.w = quat[3]

        self._tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StaticFramePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

