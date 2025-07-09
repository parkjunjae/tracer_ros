import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np

class RealtimeAccumGridPublisher(Node):
    def __init__(self):
        super().__init__('realtime_accum_occupancy_grid')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lio_sam/mapping/cloud_registered',  # 누적할 토픽명
            self.callback,
            10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 1)

        # 누적 배열
        self.accum_points = []
        self.max_frames = 300  # 원하는 만큼, 예: 5Hz 기준 1분

        # 맵 파라미터
        self.origin_x = -10.0
        self.origin_y = -10.0
        self.resolution = 0.05
        self.width = 400
        self.height = 400
        self.zmin = 0.05
        self.zmax = 1.5
        self.free_value = 254
        self.occ_value = 0

    def callback(self, msg):
        # PointCloud2 → numpy 변환
        pts = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not pts:
            self.get_logger().warn("PointCloud 데이터 없음")
            return
        points = np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float32)
        # z 필터
        mask = (points[:,2] > self.zmin) & (points[:,2] < self.zmax)
        points = points[mask]

        # 누적
        self.accum_points.append(points)
        if len(self.accum_points) > self.max_frames:
            self.accum_points.pop(0)
        all_points = np.concatenate(self.accum_points, axis=0)

        # OccupancyGrid 변환
        grid = np.full((self.height, self.width), self.free_value, dtype=np.uint8)
        for p in all_points:
            x_idx = int((p[0] - self.origin_x) / self.resolution)
            y_idx = int((p[1] - self.origin_y) / self.resolution)
            if 0 <= x_idx < self.width and 0 <= y_idx < self.height:
                grid[y_idx, x_idx] = self.occ_value

        og = OccupancyGrid()
        og.header = Header()
        og.header.stamp = self.get_clock().now().to_msg()
        og.header.frame_id = 'map'
        og.info.resolution = self.resolution
        og.info.width = self.width
        og.info.height = self.height
        og.info.origin.position.x = self.origin_x
        og.info.origin.position.y = self.origin_y
        og.info.origin.position.z = 0.0
        og.info.origin.orientation.w = 1.0
        grid_ros = np.full_like(grid, -1, dtype=np.int8)
        grid_ros[grid == self.occ_value] = 0
        grid_ros[grid == self.free_value] = -1
        og.data = grid_ros.flatten().tolist()

        self.map_pub.publish(og)
        self.get_logger().info('누적 OccupancyGrid published, points=%d' % all_points.shape[0])

def main(args=None):
    rclpy.init(args=args)
    node = RealtimeAccumGridPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

