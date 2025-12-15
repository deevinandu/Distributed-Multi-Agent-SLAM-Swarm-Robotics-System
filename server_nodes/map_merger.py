import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
import numpy as np
import open3d as o3d
import copy

class MapMerger(Node):
    def __init__(self):
        super().__init__('map_merger')

        self.declare_parameter('agent_ids', [1])
        self.agent_ids = self.get_parameter('agent_ids').get_parameter_value().integer_array_value
        if not self.agent_ids: 
             self.agent_ids = [1]

        self.subscriptions_ = []
        for agent_id in self.agent_ids:
            topic = f'/agent_{agent_id}/map'
            self.get_logger().info(f'Subscribing to {topic}')
            sub = self.create_subscription(
                OccupancyGrid,
                topic,
                lambda msg, aid=agent_id: self.map_callback(msg, aid),
                10
            )
            self.subscriptions_.append(sub)

        self.publisher_ = self.create_publisher(OccupancyGrid, '/global_map', 10)

        self.global_pcd = o3d.geometry.PointCloud()
        self.map_resolution = 0.05 
        self.map_origin = [0.0, 0.0]

    def map_callback(self, msg: OccupancyGrid, agent_id: int):
        local_pcd = self.grid_to_pcd(msg)
        if local_pcd.is_empty():
            return

        if self.global_pcd.is_empty():
            self.global_pcd = local_pcd
            self.map_resolution = msg.info.resolution
            self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
        else:
            trans_init = np.identity(4)
            threshold = 1.0 
            
            reg_p2p = o3d.pipelines.registration.registration_icp(
                local_pcd, self.global_pcd, threshold, trans_init,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=30)
            )
            
            if reg_p2p.fitness < 0.6: 
                self.get_logger().warn(f"ICP fitness low ({reg_p2p.fitness}). Rejecting.")
                return

            local_pcd.transform(reg_p2p.transformation)
            self.global_pcd += local_pcd
            self.global_pcd = self.global_pcd.voxel_down_sample(voxel_size=self.map_resolution)

        self.publish_global_map(msg.header.frame_id)

    def grid_to_pcd(self, msg: OccupancyGrid) -> o3d.geometry.PointCloud:
        width = msg.info.width
        height = msg.info.height
        res = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        data = np.array(msg.data).reshape((height, width))
        occ_indices = np.argwhere(data > 50) 

        points = []
        if occ_indices.size > 0:
            y_coords = occ_indices[:, 0] * res + origin_y
            x_coords = occ_indices[:, 1] * res + origin_x
            z_coords = np.zeros_like(x_coords)
            points = np.vstack((x_coords, y_coords, z_coords)).T

        pcd = o3d.geometry.PointCloud()
        if len(points) > 0:
            pcd.points = o3d.utility.Vector3dVector(points)
        
        return pcd

    def publish_global_map(self, frame_id):
        if self.global_pcd.is_empty():
            return

        points = np.asarray(self.global_pcd.points)
        if len(points) == 0:
            return

        min_x = np.min(points[:, 0])
        max_x = np.max(points[:, 0])
        min_y = np.min(points[:, 1])
        max_y = np.max(points[:, 1])

        width = int(np.ceil((max_x - min_x) / self.map_resolution)) + 1
        height = int(np.ceil((max_y - min_y) / self.map_resolution)) + 1

        grid_data = np.full((height, width), -1, dtype=np.int8) 

        x_indices = ((points[:, 0] - min_x) / self.map_resolution).astype(int)
        y_indices = ((points[:, 1] - min_y) / self.map_resolution).astype(int)

        x_indices = np.clip(x_indices, 0, width - 1)
        y_indices = np.clip(y_indices, 0, height - 1)

        grid_data[y_indices, x_indices] = 100

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map_global"

        msg.info.resolution = self.map_resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = min_x
        msg.info.origin.position.y = min_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = grid_data.flatten().tolist()

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MapMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
