#!/usr/bin/env python3
import math
import os
from typing import List, Tuple

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from std_srvs.srv import Empty


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def bresenham(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
    points = []
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    while True:
        points.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy
    return points


class SimpleMapper(Node):
    def __init__(self):
        super().__init__("simple_mapper")

        # ---- Map params ----
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("width", 400)
        self.declare_parameter("height", 400)
        self.declare_parameter("origin_x", -10.0)
        self.declare_parameter("origin_y", -10.0)

        # To avoid needing map->odom TF, set frame_id := "odom" (recommended for simple projects)
        self.declare_parameter("frame_id", "odom")

        self.declare_parameter("publish_period", 0.5)

        # Log-odds params
        self.declare_parameter("lo_occ", 0.85)
        self.declare_parameter("lo_free", 0.40)
        self.declare_parameter("lo_min", -6.0)
        self.declare_parameter("lo_max",  6.0)

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")

        self.declare_parameter("save_dir", os.path.expanduser("~/maps"))
        self.declare_parameter("map_name", "my_map")

        self.res = float(self.get_parameter("resolution").value)
        self.w = int(self.get_parameter("width").value)
        self.h = int(self.get_parameter("height").value)
        self.ox = float(self.get_parameter("origin_x").value)
        self.oy = float(self.get_parameter("origin_y").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.lo_occ = float(self.get_parameter("lo_occ").value)
        self.lo_free = float(self.get_parameter("lo_free").value)
        self.lo_min = float(self.get_parameter("lo_min").value)
        self.lo_max = float(self.get_parameter("lo_max").value)

        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)

        self.log_odds = [0.0] * (self.w * self.h)
        self.seen = [False] * (self.w * self.h)

        self.robot_pose = None  # (x, y, yaw)

        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 50)
        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 50)

        self.map_pub = self.create_publisher(OccupancyGrid, "/map", 10)

        pub_period = float(self.get_parameter("publish_period").value)
        self.create_timer(pub_period, self.publish_map)

        self.create_service(Empty, "/save_map", self.on_save_map)

        self.get_logger().info("SimpleMapper: building /map from /scan + /odom (no SLAM libs).")

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        gx = int((x - self.ox) / self.res)
        gy = int((y - self.oy) / self.res)
        return gx, gy

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.w and 0 <= gy < self.h

    def idx(self, gx: int, gy: int) -> int:
        return gy * self.w + gx

    def clamp(self, v: float) -> float:
        return max(self.lo_min, min(self.lo_max, v))

    def add_free(self, gx: int, gy: int):
        if not self.in_bounds(gx, gy):
            return
        i = self.idx(gx, gy)
        self.log_odds[i] = self.clamp(self.log_odds[i] - self.lo_free)
        self.seen[i] = True

    def add_occ(self, gx: int, gy: int):
        if not self.in_bounds(gx, gy):
            return
        i = self.idx(gx, gy)
        self.log_odds[i] = self.clamp(self.log_odds[i] + self.lo_occ)
        self.seen[i] = True

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        self.robot_pose = (p.x, p.y, yaw)

    def on_scan(self, msg: LaserScan):
        if self.robot_pose is None:
            return

        rx, ry, rth = self.robot_pose
        start_gx, start_gy = self.world_to_grid(rx, ry)
        if not self.in_bounds(start_gx, start_gy):
            return

        angle = msg.angle_min
        for r in msg.ranges:
            if math.isinf(r) or math.isnan(r):
                angle += msg.angle_increment
                continue

            hit = True
            if r >= msg.range_max:
                r = msg.range_max
                hit = False

            beam = rth + angle
            ex = rx + r * math.cos(beam)
            ey = ry + r * math.sin(beam)
            end_gx, end_gy = self.world_to_grid(ex, ey)

            line = bresenham(start_gx, start_gy, end_gx, end_gy)

            for gx, gy in line[:-1]:
                self.add_free(gx, gy)

            if hit:
                self.add_occ(end_gx, end_gy)

            angle += msg.angle_increment

    def lo_to_occ(self, lo: float, seen: bool) -> int:
        if not seen:
            return -1
        p = 1.0 - 1.0 / (1.0 + math.exp(lo))
        if p > 0.65:
            return 100
        if p < 0.35:
            return 0
        return -1

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()

        info = MapMetaData()
        info.resolution = self.res
        info.width = self.w
        info.height = self.h
        info.origin.position.x = self.ox
        info.origin.position.y = self.oy
        info.origin.orientation.w = 1.0
        msg.info = info

        msg.data = [self.lo_to_occ(self.log_odds[i], self.seen[i]) for i in range(self.w * self.h)]
        self.map_pub.publish(msg)

    def on_save_map(self, request, response):
        save_dir = str(self.get_parameter("save_dir").value)
        name = str(self.get_parameter("map_name").value)
        os.makedirs(save_dir, exist_ok=True)

        pgm_path = os.path.join(save_dir, f"{name}.pgm")
        yaml_path = os.path.join(save_dir, f"{name}.yaml")

        with open(pgm_path, "w", encoding="utf-8") as f:
            f.write("P2\n")
            f.write(f"{self.w} {self.h}\n")
            f.write("255\n")
            for y in range(self.h - 1, -1, -1):
                row = []
                for x in range(self.w):
                    i = self.idx(x, y)
                    occ = self.lo_to_occ(self.log_odds[i], self.seen[i])
                    if occ == -1:
                        val = 205      # unknown
                    elif occ == 0:
                        val = 254      # free (light)
                    else:
                        val = 0        # occupied (dark)
                    row.append(str(val))
                f.write(" ".join(row) + "\n")

        with open(yaml_path, "w", encoding="utf-8") as f:
            f.write(f"image: {os.path.basename(pgm_path)}\n")
            f.write(f"resolution: {self.res}\n")
            f.write(f"origin: [{self.ox}, {self.oy}, 0.0]\n")
            f.write("negate: 0\n")
            f.write("occupied_thresh: 0.65\n")
            f.write("free_thresh: 0.35\n")

        self.get_logger().info(f"Saved map: {pgm_path} and {yaml_path}")
        return response


def main():
    rclpy.init()
    node = SimpleMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

