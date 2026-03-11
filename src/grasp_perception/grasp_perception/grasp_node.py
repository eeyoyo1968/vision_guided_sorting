#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float32

import subprocess
import threading
import json
import sys
import os


SCRIPT_PATH = "/home/raico/grasp/runtime/scripts/live_worldframe_yolo_master_v5_1.py"


class GraspPerceptionNode(Node):

    def __init__(self):
        super().__init__("grasp_perception_node")

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, "/grasp/pose", 10)
        self.status_pub = self.create_publisher(String, "/grasp/status", 10)
        self.class_pub = self.create_publisher(String, "/grasp/class", 10)
        self.conf_pub = self.create_publisher(Float32, "/grasp/conf", 10)
        self.width_pub = self.create_publisher(Float32, "/grasp/width", 10)

        self.get_logger().info("Starting perception subprocess:")
        self.get_logger().info(f"  python3 {SCRIPT_PATH}")

        self.proc = subprocess.Popen(
            ["python3", SCRIPT_PATH],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1
        )

        self.reader_thread = threading.Thread(
            target=self.read_stdout_loop,
            daemon=True
        )
        self.reader_thread.start()

    # ===============================
    # Subprocess reader
    # ===============================

    def read_stdout_loop(self):
        for line in self.proc.stdout:
            line = line.strip()
            if not line:
                continue

            try:
                data = json.loads(line)
                self.publish_from_json(data)
            except Exception:
                # Ignore non-JSON lines
                continue

    # ===============================
    # Publish ROS messages
    # ===============================

    def publish_from_json(self, data):

        status_msg = String()
        status_msg.data = data.get("status", "UNKNOWN")
        self.status_pub.publish(status_msg)

        if status_msg.data != "OK":
            return

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = data.get("frame_id", "table_world")

        xyz = data.get("xyz_world_m", [0.0, 0.0, 0.0])
        quat = data.get("quat_xyzw", [0.0, 0.0, 0.0, 1.0])

        pose.pose.position.x = float(xyz[0])
        pose.pose.position.y = float(xyz[1])
        pose.pose.position.z = float(xyz[2])

        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])

        self.pose_pub.publish(pose)

        class_msg = String()
        class_msg.data = data.get("class", "unknown")
        self.class_pub.publish(class_msg)

        conf_msg = Float32()
        conf_msg.data = float(data.get("conf", 0.0))
        self.conf_pub.publish(conf_msg)

        width_msg = Float32()
        width_msg.data = float(data.get("width_m", 0.0))
        self.width_pub.publish(width_msg)

    # ===============================
    # Clean shutdown
    # ===============================

    def destroy_node(self):
        self.get_logger().info("Shutting down perception subprocess...")

        if self.proc is not None:
            try:
                self.proc.terminate()
            except Exception:
                pass

        super().destroy_node()


# ===============================
# Main
# ===============================

def main(args=None):
    rclpy.init(args=args)
    node = GraspPerceptionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

