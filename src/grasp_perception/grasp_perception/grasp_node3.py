#!/usr/bin/env python3
import os
import sys
import json
import signal
import subprocess
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32, Int32
from geometry_msgs.msg import PoseStamped

# Optional TF publisher
try:
    from tf2_ros import TransformBroadcaster
    from geometry_msgs.msg import TransformStamped
    HAVE_TF = True
except Exception:
    HAVE_TF = False


class GraspNode3(Node):
    """
    GraspNode3:
    - Spawns perception script as subprocess (venv python)
    - Reads JSON lines from stdout
    - Publishes grasp pose + pregrasp pose + debug topics
    """

    def __init__(self):
        super().__init__('graspnode3')

        # ---------- Parameters ----------
        self.declare_parameter('perception_python', '/home/raico/grasp_vision/.venv/bin/python3')
        self.declare_parameter('perception_script', '/home/raico/grasp/runtime/scripts/live_worldframe_yolo_master_v5_6.py')
        self.declare_parameter('frame_id', 'table_world')
        self.declare_parameter('use_tf', False)
        self.declare_parameter('tf_child_frame', 'grasp_target')
        self.declare_parameter('auto_restart', True)

        self.perception_python = self.get_parameter('perception_python').value
        self.perception_script = self.get_parameter('perception_script').value
        self.frame_id = self.get_parameter('frame_id').value
        self.use_tf = bool(self.get_parameter('use_tf').value)
        self.tf_child_frame = self.get_parameter('tf_child_frame').value
        self.auto_restart = bool(self.get_parameter('auto_restart').value)

        # ---------- Publishers ----------
        self.pub_status = self.create_publisher(String, '/grasp/status', 10)
        self.pub_class = self.create_publisher(String, '/grasp/class', 10)
        self.pub_conf = self.create_publisher(Float32, '/grasp/conf', 10)
        self.pub_score = self.create_publisher(Float32, '/grasp/score', 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/grasp/pose', 10)
        self.pub_pre_pose = self.create_publisher(PoseStamped, '/grasp/pre_pose', 10)
        self.pub_width = self.create_publisher(Float32, '/grasp/width', 10)
        self.pub_plane_std = self.create_publisher(Float32, '/grasp/plane_std', 10)
        self.pub_plane_inliers = self.create_publisher(Int32, '/grasp/plane_inliers', 10)

        # ---------- TF (optional) ----------
        self.tf_broadcaster = TransformBroadcaster(self) if (self.use_tf and HAVE_TF) else None

        # ---------- Subprocess + read timer ----------
        self.proc: Optional[subprocess.Popen] = None
        self._start_perception()

        # 100 Hz read loop: non-blocking-ish with readline() (pipe buffered)
        self.timer = self.create_timer(0.01, self._read_loop)

        # Health check timer
        self.health_timer = self.create_timer(1.0, self._health_check)

        self.get_logger().info("GraspNode3 started.")
        self.get_logger().info(f"  perception_python: {self.perception_python}")
        self.get_logger().info(f"  perception_script: {self.perception_script}")
        self.get_logger().info(f"  frame_id: {self.frame_id}")
        self.get_logger().info(f"  use_tf: {self.use_tf} (available={HAVE_TF})")

    def _start_perception(self):
        # Validate files
        if not os.path.exists(self.perception_python):
            self.get_logger().error(f"perception_python not found: {self.perception_python}")
        if not os.path.exists(self.perception_script):
            self.get_logger().error(f"perception_script not found: {self.perception_script}")

        # Ensure unbuffered prints from perception script
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"

        cmd = [self.perception_python, self.perception_script]

        self.get_logger().info("Starting perception subprocess:")
        self.get_logger().info("  " + " ".join(cmd))

        try:
            self.proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                env=env
            )
        except Exception as e:
            self.get_logger().error(f"Failed to start perception subprocess: {e}")
            self.proc = None

    def _stop_perception(self):
        if self.proc is None:
            return

        try:
            if self.proc.poll() is None:
                self.get_logger().info("Stopping perception subprocess...")
                self.proc.send_signal(signal.SIGINT)
                try:
                    self.proc.wait(timeout=2.0)
                except Exception:
                    self.proc.kill()
        except Exception as e:
            self.get_logger().warn(f"Error stopping subprocess: {e}")
        finally:
            self.proc = None

    def _health_check(self):
        # If process died, optionally restart
        if self.proc is None:
            if self.auto_restart:
                self.get_logger().warn("Perception process missing -> restarting...")
                self._start_perception()
            return

        rc = self.proc.poll()
        if rc is not None:
            self.get_logger().warn(f"Perception process exited (code={rc}).")
            self.proc = None
            if self.auto_restart:
                self.get_logger().warn("Restarting perception...")
                self._start_perception()

    def _read_loop(self):
        if self.proc is None or self.proc.stdout is None:
            return

        # Read as many lines as available quickly
        # NOTE: readline() blocks if nothing is available, but timer runs often;
        # pipe is line-buffered; perception prints frequently (~10Hz+).
        try:
            line = self.proc.stdout.readline()
        except Exception:
            return

        if not line:
            return

        line = line.strip()
        if not line:
            return

        # Perception prints JSON lines; but may also print logs -> ignore non-JSON
        if not (line.startswith("{") and line.endswith("}")):
            # Optional: forward log lines to ros log
            # self.get_logger().info(f"[perception] {line}")
            return

        try:
            msg = json.loads(line)
        except Exception:
            return

        self._publish_from_json(msg)

    def _publish_from_json(self, d: dict):
        # Extract fields robustly
        status = str(d.get("status", ""))
        cls = str(d.get("class", ""))
        conf = float(d.get("conf", 0.0) or 0.0)
        score = float(d.get("score", 0.0) or 0.0)
        xyz = d.get("xyz_world_m", [0.0, 0.0, 0.0])
        pre_xyz = d.get("pre_xyz_world_m", [0.0, 0.0, 0.0])
        quat = d.get("quat_xyzw", [0.0, 0.0, 0.0, 1.0])
        width = float(d.get("width_m", 0.0) or 0.0)
        plane_std = float(d.get("plane_std_m", 0.0) or 0.0)
        plane_inliers = int(d.get("plane_inliers", 0) or 0)

        # Stamp
        now = self.get_clock().now().to_msg()

        # status
        sm = String()
        sm.data = status
        self.pub_status.publish(sm)

        cm = String()
        cm.data = cls
        self.pub_class.publish(cm)

        fm = Float32()
        fm.data = conf
        self.pub_conf.publish(fm)

        sc = Float32()
        sc.data = score
        self.pub_score.publish(sc)

        wm = Float32()
        wm.data = width
        self.pub_width.publish(wm)

        ps = Float32()
        ps.data = plane_std
        self.pub_plane_std.publish(ps)

        pi = Int32()
        pi.data = plane_inliers
        self.pub_plane_inliers.publish(pi)

        # pose
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = self.frame_id

        pose.pose.position.x = float(xyz[0])
        pose.pose.position.y = float(xyz[1])
        pose.pose.position.z = float(xyz[2])

        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])

        self.pub_pose.publish(pose)

        # pre pose
        pre = PoseStamped()
        pre.header.stamp = now
        pre.header.frame_id = self.frame_id

        pre.pose.position.x = float(pre_xyz[0])
        pre.pose.position.y = float(pre_xyz[1])
        pre.pose.position.z = float(pre_xyz[2])

        pre.pose.orientation = pose.pose.orientation  # same orientation
        self.pub_pre_pose.publish(pre)

        # TF (optional)
        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.frame_id
            t.child_frame_id = self.tf_child_frame

            t.transform.translation.x = pose.pose.position.x
            t.transform.translation.y = pose.pose.position.y
            t.transform.translation.z = pose.pose.position.z

            t.transform.rotation = pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        self._stop_perception()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GraspNode3()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
