from kinova_controller.kinova import KinovaArm
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

retract_xyz = [0.12, 0.0, 0.31]
retract_quat = [0.7, 0.7, 0.0, 0.0] # identity orientation (x, y, z, w)

home_xyz = [0.55, 0.0, 0.35]
home_quat = [0.5, 0.5, 0.5, 0.5]

low_xyz = [0.45, -0.10867, 0.14407]
low_quat = [0.49996, 0.49982, 0.50035, 0.49987]


class TargetPoseListener(Node):
    def __init__(self):
        super().__init__("target_pose_listener")
        self.latest_pose = None
        self.create_subscription(PoseStamped, "/button/target_pose", self.cb_pose, 10)

    def cb_pose(self, msg: PoseStamped):
        self.latest_pose = msg


def wait_for_target_pose(timeout_s: float = 10.0):
    rclpy.init()
    node = TargetPoseListener()
    try:
        start_t = time.time()
        while rclpy.ok() and (time.time() - start_t) < timeout_s:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.latest_pose is not None:
                p = node.latest_pose.pose.position
                q = node.latest_pose.pose.orientation
                xyz = [p.x, p.y, p.z]
                quat = [q.x, q.y, q.z, q.w]
                return xyz, quat
        return None, None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    arm = KinovaArm()
    try:
        hard_limits = arm.control_config.GetKinematicHardLimits()
        speed_limits = tuple(0.7 * v for v in hard_limits.joint_speed_limits)
        acceleration_limits = tuple(0.7 * v for v in hard_limits.joint_acceleration_limits)
        arm.set_joint_limits(speed_limits=speed_limits, acceleration_limits=acceleration_limits)
        arm.set_twist_linear_limit(0.7 * hard_limits.twist_linear)
        # arm.move_cartesian(low_xyz, low_quat, blocking=True)
        arm.move_cartesian(home_xyz, home_quat, blocking=True)
        prev_target_xyz = None
        while True:
            target_xyz, target_quat = wait_for_target_pose(timeout_s=10.0)
            if target_xyz is None:
                raise RuntimeError("Timed out waiting for /button/target_pose from yolo.py")
            if prev_target_xyz is not None:
                dx = target_xyz[0] - prev_target_xyz[0]
                dy = target_xyz[1] - prev_target_xyz[1]
                dz = target_xyz[2] - prev_target_xyz[2]
                if (dx * dx + dy * dy + dz * dz) ** 0.5 < 0.01:
                    break
            prev_target_xyz = target_xyz
        # Move toward button non-blocking so we can monitor force
        FORCE_THRESHOLD = 20.0  # Newtons – tune this for your button
        PUSH_TIMEOUT = 10.0    # seconds
        PUSH_EXTRA = 0.02      # meters to overshoot past the button surface

        # Overshoot the target so the arm pushes through the button
        push_target_xyz = list(target_xyz)
        push_target_xyz[0] += PUSH_EXTRA

        arm.move_cartesian(push_target_xyz, target_quat, blocking=False)

        start_t = time.time()
        while time.time() - start_t < PUSH_TIMEOUT:
            # Check if the action already finished (reached target or aborted)
            if arm.ready():
                break
            force = arm.get_ee_force()
            force_mag = float(np.linalg.norm(force))
            if force_mag > FORCE_THRESHOLD:
                print(f"Contact detected! Force magnitude: {force_mag:.1f} N")
                break
            time.sleep(0.02)

        # Stop the push motion, clear any faults, then retract
        arm.stop_action()
        time.sleep(0.5)
        arm.clear_faults()
        arm.end_or_abort_event.set()

        arm.move_cartesian(retract_xyz, retract_quat, blocking=True)
    finally:
        arm.disconnect()

if __name__ == "__main__":
    main()
