import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
import numpy as np
import open3d as o3d 
from .tools import*
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException
from tf2_ros import TransformBroadcaster


class Pointcloudloc(Node):

    def __init__(self):
        super().__init__('icp_localization')
        
        self.params = self.declare_parameters(
            namespace='',
            parameters=[
                ('voxelSize', None),
                ('threshold', None),
                ('iteration_max', None),
                ('transformation_init.use_global_loc', None),
                ('transformation_init.init_trans', None),
                ('relative_fitness', None), 
                ('relative_rmse', None), 
                ('icp_normal_rad', None)
            ])
        self.map = None
        # self.pcd = o3d.geometry.PointCloud()
        self.tf_v2r = None
        self.tf_I2r = None
        self.first = True
        self.imu_init = True
        self.use_global_loc = self.params[3].value
        self.init_transform = None
        self.icp_threshold = self.params[1].value
        self.max_iteration = self.params[2].value
        self.voxel_size = self.params[0].value
        self.relative_fitness = self.params[5].value
        self.relative_rmse = self.params[6].value
        self.icp_rad = self.params[7].value
        self.transform = None
        self.imu_reset = None
        self.prev_imu = np.identity(4)
        self.imu_rotation = np.identity(4) 
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.pcd_publisher = self.create_publisher(sensor_msgs.PointCloud2, '/velo_pcd', 10)
        # self.vis = o3d.visualization.Visualizer()
        # self.vis.create_window(window_name='Debugging', width=1500, height=800)
        assert len(sys.argv) > 1, "No pcd file given."
        assert os.path.exists(sys.argv[1]), "File doesn't exist."
        map_path = sys.argv[1]
        self.map = o3d.io.read_point_cloud(map_path)
        self.map.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_rad, max_nn=self.max_iteration)) 
        # self.map.paint_uniform_color([1, 1, 1])
        # self.vis.add_geometry(self.map)
        self.subscription = self.create_subscription(PointCloud2,'/velodyne_points',self.pointcloud_callback,10)
        self.subscription  
        self.imu_subscription = self.create_subscription(Imu,'/imu',self.imu_callback,10)
        self.imu_subscription
        # self.vis.poll_events()
        # self.vis.update_renderer()
        
    def pointcloud_callback(self, msg):
        time = self.get_clock().now()
        # self.vis.remove_geometry(self.pcd)
        points_as_array = np.array(list(read_points(msg, skip_nans=True))) # point cloud as np.array
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_as_array)
        if self.first:
            if self.use_global_loc: 
                self.init_transform = execute_fast_global_registration(self.voxel_size, pcd, self.map)
            else:
                self.init_transform = np.asarray(self.params[4].value).reshape((4,4))
            try:
                t = self.tf_buffer.lookup_transform('robot_base','velodyneLidar',rclpy.time.Time())
                p,q = tf_to_pq(t)
                self.tf_v2r = pq_2_trans(p,q)
                self.get_logger().info('transform recived')
            except TransformException as ex:
                    self.get_logger().info(
                        f'Could not transform robot_base to velodyneLidar: {ex}')
            self.first = False
        pcd.transform(self.tf_v2r)
        # self.pcd  = pcd
        # self.vis.add_geometry(self.pcd)
        self.init_transform =  self.init_transform  @ self.imu_rotation 
        self.transform = self.register(pcd)
        self.init_transform = self.transform
        self.get_logger().info(f'Registeration :  {self.get_clock().now()-time}')
        self.get_logger().info(f'Transform: \n {self.transform}')
        self.creat_tf(self.transform)
        cloud = point_cloud(np.asarray(pcd.points), 'robot_base')
        self.pcd_publisher.publish(cloud)
        self.get_logger().info('=======================')
        self.get_logger().info('\n')
        # self.vis.poll_events()
        # self.vis.update_renderer()
        
    def register(self, scan):
        scan.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=self.icp_rad, max_nn=self.max_iteration)) 
        result_icp = o3d.pipelines.registration.registration_icp(
                        scan,
                        self.map,
                        self.icp_threshold,
                        self.init_transform,
                        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                        o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness = self.relative_fitness, relative_rmse = self.relative_rmse,max_iteration=self.max_iteration)
                        )
        return result_icp.transformation
    
    
    def imu_callback(self,msg):
        if self.imu_init:
            translation = np.array([0, 0 , 0])
            quat = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            T_init = pq_2_trans(translation,quat)
            self.imu_reset = np.linalg.inv(T_init)
            self.imu_init = 0

        translation = np.array([0, 0 , 0])
        quat = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        T = self.imu_reset @ pq_2_trans(translation,quat)
        self.imu_rotation = np.linalg.inv(self.prev_imu) @T
        self.imu_rotation[:3, 3] = [0, 0, 0]
        self.get_logger().info(f'IMU: \n {self.imu_rotation}')
        self.prev_imu = T
        
        
        
                  
    def creat_tf(self,tf):
        ttf = tf.copy() 
        m = ttf[:3,:3]
        p = ttf[:3, 3]
        r = R.from_matrix(m)
        q = r.as_quat()
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'Map'
        t.child_frame_id = 'robot_base'
        t.transform.translation.x = p[0]
        t.transform.translation.y = p[1]
        t.transform.translation.z = p[2]
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)
        
    def debuging_tap(self, pcd): 
        pass 

def main(args=None):
    rclpy.init(args=args)

    localize = Pointcloudloc()

    rclpy.spin(localize)

    localize.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

