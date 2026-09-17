#!/usr/bin/env python3
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import math

class IMUProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')
        
        self.subscription = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        
        self.publisher = self.create_publisher(Float32, '/imu/slope', 10)
        
        self.get_logger().info('✅ IMU Processor started — listening on /imu/data')

    def imu_callback(self, msg):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        
        try:
            slope_angle = math.atan2(math.sqrt(ax**2 + ay**2), abs(az))
        except:
            slope_angle = 0.0
            
        self.publisher.publish(Float32(data=float(slope_angle)))
        self.get_logger().debug(f'Slope: {math.degrees(slope_angle):.2f}°')

def main(args=None):
    rclpy.init(args=args)
    node = IMUProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
