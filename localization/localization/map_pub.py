import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import open3d as o3d 
from .tools import*


def main(args=None):
    # Boilerplate code.
    rclpy.init(args=args)
    pcd_publisher = MapPublisher()
    rclpy.spin(pcd_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pcd_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()