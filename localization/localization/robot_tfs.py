import math
import sys
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class StaticFramePublisher(Node):
    """
    Broadcast transforms that never change.
    """

    def __init__(self, velodyne,basler,board):
        super().__init__('robot_tfs')

        tf_static_broadcaster1 = StaticTransformBroadcaster(self)
        tf_static_broadcaster2 = StaticTransformBroadcaster(self)
        tf_static_broadcaster3 = StaticTransformBroadcaster(self)
        # Publish static transforms once at startup
        self.make_transforms(velodyne, tf_static_broadcaster1)
        self.make_transforms(basler,tf_static_broadcaster2)
        self.make_transforms(board,tf_static_broadcaster3)
        
    def make_transforms(self, transformation,tf):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'robot_base'
        t.child_frame_id = transformation[7]
        t.transform.translation.x = float(transformation[0])
        t.transform.translation.y = float(transformation[1])
        t.transform.translation.z = float(transformation[2])
        t.transform.rotation.x = float(transformation[3])
        t.transform.rotation.y = float(transformation[4])
        t.transform.rotation.z = float(transformation[5])
        t.transform.rotation.w = float(transformation[6])
        tf.sendTransform(t)


def main():
    
    velodyne = ['-0.19','0.613','0','0','0','0','1.0','velodyneLidar']
    basler = ['0.126104','0.00069','0.46668','0','0','0','1.0','baslerCamera']
    board = ['-0.13105','-0.0202','0.26839','0','0','0','1.0','Board']
    rclpy.init()
    node = StaticFramePublisher(velodyne,basler,board)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
