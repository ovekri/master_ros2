import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from std_msgs.msg import Int32MultiArray

class LidarControl(Node):
    def __init__(self):
        super().__init__('lidar_control')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/LiDAR_transformed',
            self.reader_callback,
            1)
        self.subscription
        self.publisher = self.create_publisher(Int32MultiArray, 'lidar_decision', 1)

    def reader_callback(self, cloud_msg):
            
        # process the point cloud
        xyz_points = list(read_points(cloud_msg, field_names=('x', 'y', 'z'), skip_nans=True))
        counter_tot = 0
        counter_approved = 0
        counter_to_close = 0
        to_close_ratio = 0
        for point in xyz_points:
            counter_tot += 1
            if (point[2] < 0.2 and point[2] > 0.0): # height
                if (point[1] < 0.2 and point[1] > -0.2): # width
                    counter_approved += 1
                    if (point[0] < 1): # distance 
                        counter_to_close += 1
                        #print(f"Transformed point coordinates: X: {point[0]}, Y: {point[1]}, Z: {point[2]}")
        to_close_ratio = counter_to_close/counter_approved
        array_msg = Int32MultiArray()
        if (to_close_ratio < 0.1 or counter_approved < 3):
            array_msg.data = [1, 0, 0, 0]
            self.publisher.publish(array_msg)
        else:
            self.get_logger().info('DANGER: Obsticle straight ahead')
            #self.get_logger().info(f'prosentage of point to close: {to_close_ratio*100}')
            array_msg.data = [0, 0, 0, 0]
            self.publisher.publish(array_msg)
        #print("All points: ", counter_tot)
        #self.get_logger().info(f'Approved points: {counter_approved}')
        #self.get_logger().info(f'To close points: {counter_to_close}')
        #self.get_logger().info(f'To close ratio: {to_close_ratio}')
        #print("Processed one transformed PointCloud2 message.")
            
def main(args=None):
    rclpy.init(args=args)
    node = LidarControl()
    node.get_logger().info('lidar_control is running....')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()