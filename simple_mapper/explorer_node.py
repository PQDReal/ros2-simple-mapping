#!/usr/bin/env python3
import math
import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class SimpleExplorer(Node):
    def __init__(self):
        super().__init__("simple_explorer")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("forward_speed", 0.15)
        self.declare_parameter("turn_speed", 0.8)
        self.declare_parameter("stop_dist", 0.4)
        self.declare_parameter("turn_bias", 0.2)     # slight randomness
        self.declare_parameter("sector_deg", 60.0)   # front sector half-angle

        self.scan_topic = self.get_parameter("scan_topic").value
        self.cmd_topic = self.get_parameter("cmd_vel_topic").value

        self.fwd = float(self.get_parameter("forward_speed").value)
        self.turn = float(self.get_parameter("turn_speed").value)
        self.stop_dist = float(self.get_parameter("stop_dist").value)
        self.turn_bias = float(self.get_parameter("turn_bias").value)
        self.sector_deg = float(self.get_parameter("sector_deg").value)

        self.pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 20)

        self.last_turn_dir = 1.0  # 1=left, -1=right
        self.get_logger().info("SimpleExplorer: reactive wander/avoid (no Nav2).")

    def finite_min(self, arr):
        m = float("inf")
        for v in arr:
            if math.isfinite(v) and v > 0.0:
                m = min(m, v)
        return m

    def on_scan(self, msg: LaserScan):
        # Determine front sector indices
        half = math.radians(self.sector_deg)
        a_min = msg.angle_min
        inc = msg.angle_increment

        def idx_for_angle(a):
            return int((a - a_min) / inc)

        i0 = max(0, idx_for_angle(-half))
        i1 = min(len(msg.ranges) - 1, idx_for_angle(+half))

        front = msg.ranges[i0:i1+1]
        front_min = self.finite_min(front)

        cmd = Twist()

        if front_min < self.stop_dist:
            # Turn away; if blocked repeatedly, flip direction sometimes
            if random.random() < 0.15:
                self.last_turn_dir *= -1.0
            cmd.angular.z = self.last_turn_dir * self.turn * (1.0 + self.turn_bias * (random.random() - 0.5))
            cmd.linear.x = 0.0
        else:
            # Move forward, tiny random yaw to avoid “perfect loops”
            cmd.linear.x = self.fwd
            cmd.angular.z = self.turn_bias * (random.random() - 0.5) * 0.2

        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = SimpleExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

