import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
import numpy as np

class LidarControl(Node):
    def __init__(self):
        super().__init__('lidar_control')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/LiDAR_transformed',
            self.reader_callback,
            1)

    def reader_callback(self, cloud_msg):
            
        # process the point cloud
        xyz_points = list(read_points(cloud_msg, field_names=('x', 'y', 'z'), skip_nans=False))
        counter_tot = 0
        counter_approved = 0
        counter_to_close = 0
        to_close_ratio = 0
        for point in xyz_points:
            counter_tot += 1
                
            if (point[2] < 0.3 and point[2] > 0): # z-axis = height
                if (point[1] < 0.2 and point[1] > -0.2): # y-axis = width
                    counter_approved += 1
                    if (point[0] < 0.7): # x-axis = distance 
                        counter_to_close += 1
                        #print(f"Transformed point coordinates: X: {point[0]}, Y: {point[1]}, Z: {point[2]}")
        to_close_ratio = counter_to_close/counter_approved
        if (to_close_ratio > 0.05):
            print("Obsticle straight ahead")
            print("prosentage of point to close: ", to_close_ratio*100)
        #print("All points: ", counter_tot)
        #print("Approved points: ", counter_approved)
        print("Processed one transformed PointCloud2 message.")
            
def main(args=None):
    rclpy.init(args=args)
    node = LidarControl()
    print("lidar_control is running.... ")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()