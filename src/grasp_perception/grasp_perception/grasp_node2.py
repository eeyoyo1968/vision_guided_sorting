#!/usr/bin/env python3
import json
import subprocess
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped

PERCEPTION_SCRIPT = "/home/raico/grasp/runtime/scripts/live_worldframe_yolo_master_v5_5.py"


class GraspPerceptionNode2(Node):
    def __init__(self):
        super().__init__("grasp_perception_node2")

        self.pub_status = self.create_publisher(String, "/grasp/status", 10)
        self.pub_class  = self.create_publisher(String, "/grasp/class", 10)
        self.pub_score  = self.create_publisher(Float32, "/grasp/score", 10)
        self.pub_width  = self.create_publisher(Float32, "/grasp/width", 10)

        self.pub_pose   = self.create_publisher(PoseStamped, "/grasp/pose", 10)
        self.pub_pre    = self.create_publisher(PoseStamped, "/grasp/pre_pose", 10)

        self.get_logger().info("Starting perception subprocess:")
        self.get_logger().info(f"  python3 {PERCEPTION_SCRIPT}")

        self.proc = subprocess.Popen(
            ["python3", PERCEPTION_SCRIPT],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            universal_newlines=True,
        )

        self.timer = self.create_timer(0.01, self.poll_stdout)

    def poll_stdout(self):
        if self.proc.poll() is not None:
            self.get_logger().error("Perception subprocess exited.")
            return

        for _ in range(5):
            line = self.proc.stdout.readline()
            if not line:
                return
            line = line.strip()

            # ignore non-json prints
            if not line.startswith("{"):
                continue

            try:
                d = json.loads(line)
            except Exception:
                continue

            self.publish_from_json(d)

    def publish_from_json(self, d):
        frame_id = d.get("frame_id", "table_world")

        # status
        st = String()
        st.data = str(d.get("status", "NO_TARGET"))
        self.pub_status.publish(st)

        cl = String()
        cl.data = str(d.get("class", ""))
        self.pub_class.publish(cl)

        sc = Float32()
        sc.data = float(d.get("score", 0.0))
        self.pub_score.publish(sc)

        wd = Float32()
        wd.data = float(d.get("width_m", 0.0))
        self.pub_width.publish(wd)

        # pose
        p = PoseStamped()
        p.header.frame_id = frame_id
        p.header.stamp = self.get_clock().now().to_msg()

        xyz = d.get("xyz_world_m", [0.0, 0.0, 0.0])
        q   = d.get("quat_xyzw", [0.0, 0.0, 0.0, 1.0])

        p.pose.position.x = float(xyz[0])
        p.pose.position.y = float(xyz[1])
        p.pose.position.z = float(xyz[2])

        p.pose.orientation.x = float(q[0])
        p.pose.orientation.y = float(q[1])
        p.pose.orientation.z = float(q[2])
        p.pose.orientation.w = float(q[3])
        self.pub_pose.publish(p)

        # pre pose
        pp = PoseStamped()
        pp.header.frame_id = frame_id
        pp.header.stamp = p.header.stamp

        pre = d.get("pre_xyz_world_m", [0.0, 0.0, 0.0])
        pp.pose.position.x = float(pre[0])
        pp.pose.position.y = float(pre[1])
        pp.pose.position.z = float(pre[2])

        pp.pose.orientation = p.pose.orientation
        self.pub_pre.publish(pp)


def main(args=None):
    rclpy.init(args=args)
    node = GraspPerceptionNode2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
