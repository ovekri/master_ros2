import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
from std_msgs.msg import Int32MultiArray
from math import atan2, sqrt

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

    def forward(self, xyz_point):
        counter_tot = 0
        counter_approved = 0
        counter_to_close = 0
        to_close_ratio = 0
        for point in xyz_point:
            counter_tot += 1
            if (point[2] < 0.2 and point[2] > 0.0): # height
                if (point[1] < 0.2 and point[1] > -0.2): # width
                    if (point[0] < 10.0 and point[0] > 0.0):
                        counter_approved += 1
                    if (point[0] < 2.0 and point[0] > 0.4): # distance 
                        counter_to_close += 1
                        #print(f"Transformed point coordinates: X: {point[0]}, Y: {point[1]}, Z: {point[2]}")
        if counter_approved != 0:
            to_close_ratio = counter_to_close/counter_approved
        else:
            to_close_ratio = 0
        
        #self.get_logger().info(f'forward: {counter_to_close}')
        
        return to_close_ratio, counter_approved

    def turn(self, xyz_point):
        #right_approved = 0
        right_to_close = 0
        right_turn = 0
        left_turn = 0
        #left_approved = 0
        left_to_close = 0
        r = 0
        for x, y, z in xyz_point:
            if (z < 0.2 and z > 0.0):
                theta = atan2(y, x)
                if (theta > 0.52  and theta < 0.7):
                    #left_approved += 1
                    r = sqrt(x**2 + y**2)
                    if (r < 1.1 and r > 0.4):
                        left_to_close += 1
                if (theta < -0.52  and theta > -0.7):
                    #right_approved += 1
                    r = sqrt(x**2 + y**2)
                    if (r < 1.1 and r > 0.4):
                        right_to_close += 1
        if (left_to_close < 20):
            #self.get_logger().info(f'left is clear')
            left_turn = 1
        if (right_to_close < 20):
            right_turn = 1
            #self.get_logger().info(f'right is clear')
        
        #self.get_logger().info(f'left: {left_to_close}')
        #self.get_logger().info(f'right: {right_to_close}')
        
        return right_turn, left_turn


    def reader_callback(self, cloud_msg):
            
        # process the point cloud
        xyz_points = list(read_points(cloud_msg, field_names=('x', 'y', 'z'), skip_nans=True))

        array_msg = Int32MultiArray()
        array_msg.data = [0, 0, 0]
        turn_left = 0
        turn_right = 0
        to_close_ratio, counter_approved = self.forward(xyz_points)
        turn_right, turn_left = self.turn(xyz_points)

        if (to_close_ratio < 0.2 or counter_approved < 40):
            #self.get_logger().info('No obsticle straight ahead')
            array_msg.data[0] = 1
        else:
            array_msg.data[0] = 0
            #self.get_logger().info('DANGER: Obsticle straight ahead')

        array_msg.data[1] = turn_left
        array_msg.data[2] = turn_right
        #self.get_logger().info(f'[{array_msg.data[0]},{array_msg.data[1]},{array_msg.data[2]}]')
        self.publisher.publish(array_msg)
            
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