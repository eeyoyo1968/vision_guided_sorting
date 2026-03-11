#!/usr/bin/env python3
"""
graspnode4.py

ROS2 bridge node for live_worldframe_yolo_v5_10.py

Improvements over graspnode3:
  - Non-blocking readline via select() — never blocks the ROS2 executor
  - Additional publishers for v5.10 fields:
      /grasp/jaw_axis        (Vector3Stamped) — jaw axis in world frame
      /grasp/active          (Bool)           — True only when status=READY
      /grasp/symmetric       (Bool)           — True when yaw is locked (can)
  - Default script path points to v5.10
  - Drains multiple lines per timer tick (handles burst output)
  - stderr kept separate (piped to /dev/null) so YOLO loading logs
    never pollute the JSON stdout pipe

Topics published:
  /grasp/status         (String)          — READY / TRACKING / WIDTH_REJECT / etc.
  /grasp/active         (Bool)            — True only when status==READY
  /grasp/class          (String)          — can / glove / scissor / nut_bolt
  /grasp/conf           (Float32)         — YOLO confidence
  /grasp/score          (Float32)         — composite grasp score
  /grasp/pose           (PoseStamped)     — grasp centroid + orientation
  /grasp/pre_pose       (PoseStamped)     — pre-grasp waypoint (Z+120mm)
  /grasp/width          (Float32)         — gripper opening width (metres)
  /grasp/jaw_axis       (Vector3Stamped)  — jaw axis unit vector in world frame
  /grasp/symmetric      (Bool)            — True when yaw is locked (symmetric class)
  /grasp/plane_std      (Float32)         — table plane fit std dev (metres)
  /grasp/plane_inliers  (Int32)           — table plane inlier count

Parameters:
  perception_python   — path to venv python3
  perception_script   — path to v5.10 script
  frame_id            — world frame name (default: table_world)
  use_tf              — publish TF transform for grasp pose (default: False)
  tf_child_frame      — TF child frame name (default: grasp_target)
  auto_restart        — restart perception on crash (default: True)
  ready_only_pose     — only publish pose/pre_pose when status=READY (default: True)
"""

import os
import select
import signal
import json
import subprocess
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32, Int32, Bool
from geometry_msgs.msg import PoseStamped, Vector3Stamped

try:
    from tf2_ros import TransformBroadcaster
    from geometry_msgs.msg import TransformStamped
    HAVE_TF = True
except Exception:
    HAVE_TF = False


class GraspNode4(Node):
    """
    GraspNode4 — ROS2 bridge for v5.10 grasp perception pipeline.
    Spawns perception as a subprocess, reads JSON stdout non-blocking,
    and publishes all grasp fields as typed ROS2 topics.
    """

    def __init__(self):
        super().__init__('graspnode4')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter(
            'perception_python',
            '/home/raico/grasp_vision/.venv/bin/python3'
        )
        self.declare_parameter(
            'perception_script',
            '/home/raico/grasp/runtime/scripts/live_worldframe_yolo_master_v5_10.py'
        )
        self.declare_parameter('frame_id',         'table_world')
        self.declare_parameter('use_tf',            False)
        self.declare_parameter('tf_child_frame',   'grasp_target')
        self.declare_parameter('auto_restart',      True)
        self.declare_parameter('ready_only_pose',   True)

        self.perception_python = self.get_parameter('perception_python').value
        self.perception_script = self.get_parameter('perception_script').value
        self.frame_id          = self.get_parameter('frame_id').value
        self.use_tf            = bool(self.get_parameter('use_tf').value)
        self.tf_child_frame    = self.get_parameter('tf_child_frame').value
        self.auto_restart      = bool(self.get_parameter('auto_restart').value)
        self.ready_only_pose   = bool(self.get_parameter('ready_only_pose').value)

        # ── Publishers ─────────────────────────────────────────────────────────
        self.pub_status   = self.create_publisher(String,        '/grasp/status',        10)
        self.pub_active   = self.create_publisher(Bool,          '/grasp/active',        10)
        self.pub_class    = self.create_publisher(String,        '/grasp/class',         10)
        self.pub_conf     = self.create_publisher(Float32,       '/grasp/conf',          10)
        self.pub_score    = self.create_publisher(Float32,       '/grasp/score',         10)
        self.pub_pose     = self.create_publisher(PoseStamped,   '/grasp/pose',          10)
        self.pub_pre_pose = self.create_publisher(PoseStamped,   '/grasp/pre_pose',      10)
        self.pub_width    = self.create_publisher(Float32,       '/grasp/width',         10)
        self.pub_jaw_axis = self.create_publisher(Vector3Stamped,'/grasp/jaw_axis',      10)
        self.pub_sym      = self.create_publisher(Bool,          '/grasp/symmetric',     10)
        self.pub_plane_std    = self.create_publisher(Float32,   '/grasp/plane_std',     10)
        self.pub_plane_inliers= self.create_publisher(Int32,     '/grasp/plane_inliers', 10)

        # ── Optional TF ────────────────────────────────────────────────────────
        self.tf_broadcaster = (
            TransformBroadcaster(self) if (self.use_tf and HAVE_TF) else None
        )

        # ── Subprocess ─────────────────────────────────────────────────────────
        self.proc: Optional[subprocess.Popen] = None
        self._start_perception()

        # Read loop — 100 Hz, non-blocking via select()
        self.timer = self.create_timer(0.01, self._read_loop)

        # Health check — 1 Hz
        self.health_timer = self.create_timer(1.0, self._health_check)

        self.get_logger().info('GraspNode4 started.')
        self.get_logger().info(f'  script : {self.perception_script}')
        self.get_logger().info(f'  python : {self.perception_python}')
        self.get_logger().info(f'  frame  : {self.frame_id}')
        self.get_logger().info(f'  use_tf : {self.use_tf}  (tf2 available={HAVE_TF})')
        self.get_logger().info(f'  ready_only_pose: {self.ready_only_pose}')

    # ── Subprocess lifecycle ───────────────────────────────────────────────────

    def _start_perception(self):
        for path, label in [
            (self.perception_python, 'perception_python'),
            (self.perception_script, 'perception_script'),
        ]:
            if not os.path.exists(path):
                self.get_logger().error(f'{label} not found: {path}')

        env = os.environ.copy()
        env['PYTHONUNBUFFERED'] = '1'

        cmd = [self.perception_python, self.perception_script]
        self.get_logger().info('Starting perception: ' + ' '.join(cmd))

        try:
            self.proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,   # keep YOLO logs out of JSON pipe
                text=True,
                bufsize=1,
                env=env,
            )
        except Exception as e:
            self.get_logger().error(f'Failed to start perception: {e}')
            self.proc = None

    def _stop_perception(self):
        if self.proc is None:
            return
        try:
            if self.proc.poll() is None:
                self.get_logger().info('Stopping perception subprocess...')
                self.proc.send_signal(signal.SIGINT)
                try:
                    self.proc.wait(timeout=3.0)
                except Exception:
                    self.proc.kill()
        except Exception as e:
            self.get_logger().warning(f'Error stopping subprocess: {e}')
        finally:
            self.proc = None

    def _health_check(self):
        if self.proc is None:
            if self.auto_restart:
                self.get_logger().warning('Perception missing — restarting...')
                self._start_perception()
            return
        rc = self.proc.poll()
        if rc is not None:
            self.get_logger().warning(f'Perception exited (code={rc}).')
            self.proc = None
            if self.auto_restart:
                self.get_logger().warning('Restarting perception...')
                self._start_perception()

    # ── Non-blocking read loop ─────────────────────────────────────────────────

    def _read_loop(self):
        """
        Called at 100 Hz by ROS2 timer.
        Uses select() with timeout=0 to check if data is available BEFORE
        calling readline(). This means the callback never blocks the executor,
        even if perception is momentarily slow.
        Drains up to 10 lines per tick to handle burst output.
        """
        if self.proc is None or self.proc.stdout is None:
            return

        fd = self.proc.stdout.fileno()

        for _ in range(10):   # drain up to 10 lines per tick
            ready, _, _ = select.select([fd], [], [], 0)
            if not ready:
                break   # no data right now — return immediately, don't block

            try:
                line = self.proc.stdout.readline()
            except Exception:
                break

            if not line:
                break

            line = line.strip()
            if not line:
                continue

            # Only process valid JSON lines — skip YOLO/camera log lines
            if not (line.startswith('{') and line.endswith('}')):
                continue

            try:
                data = json.loads(line)
            except Exception:
                continue

            self._publish(data)

    # ── Publish ───────────────────────────────────────────────────────────────

    def _publish(self, d: dict):
        status   = str(d.get('status', ''))
        cls      = str(d.get('class', ''))
        conf     = float(d.get('conf',  0.0) or 0.0)
        score    = float(d.get('score', 0.0) or 0.0)
        xyz      = d.get('xyz_world_m',     [0.0, 0.0, 0.0])
        pre_xyz  = d.get('pre_xyz_world_m', [0.0, 0.0, 0.0])
        quat     = d.get('quat_xyzw',       [0.0, 0.0, 0.0, 1.0])
        jaw      = d.get('jaw_axis',         [1.0, 0.0, 0.0])
        sym      = bool(d.get('symmetric_yaw_locked', False))
        width    = float(d.get('width_m',        0.0) or 0.0)
        plane_std    = float(d.get('plane_std_m',   0.0) or 0.0)
        plane_inliers= int(d.get('plane_inliers',   0)   or 0)

        is_ready = (status == 'READY')
        now = self.get_clock().now().to_msg()

        # ── Always publish ─────────────────────────────────────────────────
        sm = String();  sm.data = status;   self.pub_status.publish(sm)
        bm = Bool();    bm.data = is_ready; self.pub_active.publish(bm)
        cm = String();  cm.data = cls;      self.pub_class.publish(cm)

        fm = Float32(); fm.data = conf;     self.pub_conf.publish(fm)
        sc = Float32(); sc.data = score;    self.pub_score.publish(sc)
        wm = Float32(); wm.data = width;    self.pub_width.publish(wm)

        ps = Float32(); ps.data = plane_std;      self.pub_plane_std.publish(ps)
        pi = Int32();   pi.data = plane_inliers;  self.pub_plane_inliers.publish(pi)

        sb = Bool();    sb.data = sym;      self.pub_sym.publish(sb)

        # Jaw axis (Vector3Stamped — always useful for rviz debugging)
        jv = Vector3Stamped()
        jv.header.stamp    = now
        jv.header.frame_id = self.frame_id
        jv.vector.x = float(jaw[0])
        jv.vector.y = float(jaw[1])
        jv.vector.z = float(jaw[2])
        self.pub_jaw_axis.publish(jv)

        # ── Pose — publish always or only when READY ────────────────────────
        if (not self.ready_only_pose) or is_ready:

            pose = PoseStamped()
            pose.header.stamp    = now
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = float(xyz[0])
            pose.pose.position.y = float(xyz[1])
            pose.pose.position.z = float(xyz[2])
            pose.pose.orientation.x = float(quat[0])
            pose.pose.orientation.y = float(quat[1])
            pose.pose.orientation.z = float(quat[2])
            pose.pose.orientation.w = float(quat[3])
            self.pub_pose.publish(pose)

            pre = PoseStamped()
            pre.header.stamp    = now
            pre.header.frame_id = self.frame_id
            pre.pose.position.x = float(pre_xyz[0])
            pre.pose.position.y = float(pre_xyz[1])
            pre.pose.position.z = float(pre_xyz[2])
            pre.pose.orientation = pose.pose.orientation
            self.pub_pre_pose.publish(pre)

            # ── Optional TF ───────────────────────────────────────────────
            if self.tf_broadcaster is not None:
                t = TransformStamped()
                t.header.stamp    = now
                t.header.frame_id = self.frame_id
                t.child_frame_id  = self.tf_child_frame
                t.transform.translation.x = pose.pose.position.x
                t.transform.translation.y = pose.pose.position.y
                t.transform.translation.z = pose.pose.position.z
                t.transform.rotation      = pose.pose.orientation
                self.tf_broadcaster.sendTransform(t)

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._stop_perception()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GraspNode4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
