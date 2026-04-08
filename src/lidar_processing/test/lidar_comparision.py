import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import struct

class LidarCompareNode(Node):
    def __init__(self):
        super().__init__("lidar_compare")

        self.raw_msg = None
        self.proc_msg = None

        self.create_subscription(PointCloud2, "/lidar_pc", self.raw_cb, 10)
        self.create_subscription(PointCloud2, "/processed_lidar_pc", self.proc_cb, 10)

    def raw_cb(self, msg):
        self.raw_msg = msg
        self.try_compare()

    def proc_cb(self, msg):
        self.proc_msg = msg
        self.try_compare()

    def extract_xyz(self, msg):
        points = []
        for i in range(msg.width * msg.height):
            offset = i * msg.point_step
            x = struct.unpack_from('f', msg.data, offset + msg.fields[0].offset)[0]
            y = struct.unpack_from('f', msg.data, offset + msg.fields[1].offset)[0]
            z = struct.unpack_from('f', msg.data, offset + msg.fields[2].offset)[0]
            points.append((x, y, z))
        return np.array(points)

    def print_stats(self, raw_count, proc_count, ratio, raw_bounds, proc_bounds):
        #print(f"Raw points: {raw_count}, Processed points: {proc_count}, Ratio: {ratio:.2f}")
        #print(f"Raw bounds: {raw_bounds}, Processed bounds: {proc_bounds}")
        print(f"Ratio: {ratio:.2f}")

    def bounds(self, points):
        if len(points) == 0:
            return None
        min_bound = np.min(points, axis=0)
        max_bound = np.max(points, axis=0)
        return (min_bound, max_bound)
    
    def try_compare(self):
        if self.raw_msg is None or self.proc_msg is None:
            return

        raw_points = self.extract_xyz(self.raw_msg)
        proc_points = self.extract_xyz(self.proc_msg)

        raw_count = len(raw_points)
        proc_count = len(proc_points)
        ratio = proc_count / raw_count if raw_count else 0.0

        raw_bounds = self.bounds(raw_points)
        proc_bounds = self.bounds(proc_points)

        self.print_stats(raw_count, proc_count, ratio, raw_bounds, proc_bounds)

        self.raw_msg = None
        self.proc_msg = None

if __name__ == "__main__":
    rclpy.init()
    node = LidarCompareNode()
    rclpy.spin(node)
    rclpy.shutdown()
