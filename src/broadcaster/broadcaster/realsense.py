import rclpy                                                                                                                                        # for creating Nodes, subscribe and publish
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster                                                                         # Broadcast transformations between coordinate frames
from geometry_msgs.msg import TransformStamped                                                                                                      # message type that represents a transformation between two coordinate frames at a specific point in time
from tf_transformations import quaternion_from_euler                                                                                                # convert euler angles to quaternion angles TODO: maybe use another package for quaternion

class RealsenseBroadcast(Node):                                                                                                                        # new class that inherits from Node class
    def __init__(self):                                                                                                                             # constructor. Initialize the attributes of LiDAR_broadcast class
        super().__init__('realsense_static_broadcast')                                                                                                  # calls the constructor of the superclass (Node), Initialize the node
        self.static_broadcaster = StaticTransformBroadcaster(self)                                                                                  # Initialize the transform broadcaster
        self.timer = self.create_timer(0.1, self.transform)

    def transform(self):
        t = TransformStamped()                                                                                                                      # create an instance of the transformstamped message type
        
        t.header.stamp = self.get_clock().now().to_msg()                                                                                            # retrieve the current time as a ROS message timestamp
        t.header.frame_id = 'original_realsense_frame'                                                                                                  # Set the parent frame ID to represent the car
        t.child_frame_id = 'transformed_realsense_frame'                                                                                                # Set the child frame ID to represent the lidar
        
        t.transform.translation.x = -0.04
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.04
        q = quaternion_from_euler(0, -95 * 3.141592 / 180, 90 * 3.141592 / 180)                                                                                       # rotate -45 degrees on the z-axis
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.static_broadcaster.sendTransform(t)                                                                                                    # Send the transformation

def main():
    rclpy.init()
    node = RealsenseBroadcast()
    print("Broadcasting Realsense transformation... ")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()


# source https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Static-Broadcaster-Py.html
