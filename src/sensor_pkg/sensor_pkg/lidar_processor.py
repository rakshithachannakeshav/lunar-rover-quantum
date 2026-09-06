#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
import math
import struct

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        self.publisher = self.create_publisher(
            PointCloud2, '/terrain/pointcloud', 10)
        
        self.get_logger().info('✅ LiDAR Processor started — listening on /scan')

    def scan_callback(self, msg):
        points = []
        for i, distance in enumerate(msg.ranges):
            if math.isinf(distance) or math.isnan(distance):
                continue
            if distance < msg.range_min or distance > msg.range_max:
                continue
            
            angle = msg.angle_min + i * msg.angle_increment
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            z = 0.0
            points.append((x, y, z))

        # Create PointCloud2 message
        cloud = PointCloud2()
        cloud.header = Header(stamp=self.get_clock().now().to_msg(), frame_id='base_link')
        cloud.height = 1
        cloud.width = len(points)
        cloud.is_dense = True
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width

        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        data = []
        for (x, y, z) in points:
            data += struct.pack('fff', x, y, z)
        cloud.data = bytes(data)

        self.publisher.publish(cloud)
        self.get_logger().debug(f'Published {len(points)} points to /terrain/pointcloud')

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
