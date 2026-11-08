"""Part A of the task: find obstacles in the LiDAR scan.

Subscribes:  /scan                      sensor_msgs/LaserScan
Publishes:   /reactive/obstacle_info    std_msgs/Float32MultiArray [front, left, right] in metres
             /reactive/obstacle_zone    std_msgs/String CLEAR | FRONT | LEFT | RIGHT | BOTH
             /reactive/safety_zone      visualization_msgs/Marker (the stop zone, for RViz)
"""
from dataclasses import fields

from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String
from visualization_msgs.msg import Marker

from robot_reactive.scan_zones import classify, compute_zones, ZoneConfig


class ObstacleDetector(Node):

    def __init__(self):
        super().__init__('obstacle_detector')
        for f in fields(ZoneConfig):
            self.declare_parameter(f.name, f.default)
        self.cfg = ZoneConfig(**{f.name: self.get_parameter(f.name).value
                                 for f in fields(ZoneConfig)})
        self.stop_distance = self.declare_parameter('stop_distance', 0.35).value
        self.side_distance = self.declare_parameter('side_distance', 0.25).value

        self.info_pub = self.create_publisher(Float32MultiArray, '/reactive/obstacle_info', 10)
        self.zone_pub = self.create_publisher(String, '/reactive/obstacle_zone', 10)
        self.marker_pub = self.create_publisher(Marker, '/reactive/safety_zone', 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.get_logger().info('Obstacle detector ready, listening to /scan')

    def on_scan(self, scan: LaserScan):
        zones = compute_zones(scan.ranges, scan.angle_min, scan.angle_increment,
                              scan.range_min, scan.range_max, self.cfg)
        zone = classify(zones, self.stop_distance, self.side_distance)

        self.info_pub.publish(Float32MultiArray(data=[zones.front, zones.left, zones.right]))
        self.zone_pub.publish(String(data=zone))
        self.marker_pub.publish(self.make_marker(scan, zone))

    def make_marker(self, scan: LaserScan, zone: str) -> Marker:
        """Draw the stop zone in front of the robot: red when occupied, green when free."""
        near = self.cfg.lidar_to_front
        far = near + self.stop_distance
        half = self.cfg.half_width + self.cfg.corridor_margin

        m = Marker()
        m.header = scan.header
        m.ns, m.id = 'stop_zone', 0
        m.type, m.action = Marker.LINE_STRIP, Marker.ADD
        m.scale.x = 0.02
        m.pose.orientation.w = 1.0
        m.color.a = 1.0
        if zone == 'FRONT':
            m.color.r = 1.0
        else:
            m.color.g = 1.0
        m.points = [Point(x=x, y=y) for x, y in
                    [(near, half), (far, half), (far, -half), (near, -half), (near, half)]]
        return m


def main():
    rclpy.init()
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
