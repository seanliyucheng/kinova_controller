"""
Authors: Jimmy Wu, Rajat Kumar Jenamani
Controls the Kinova Gen3 robot arm using the Kinova API
"""

import copy
import math
import os
import queue
import subprocess
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

# This is a workaround for Kinova API using collections.MutableMapping and collections.MutableSequence
import collections
if not hasattr(collections, 'MutableMapping'):
    from collections.abc import MutableMapping
    collections.MutableMapping = MutableMapping
if not hasattr(collections, 'MutableSequence'):
    from collections.abc import MutableSequence
    collections.MutableSequence = MutableSequence

from kortex_api.autogen.client_stubs.ActuatorConfigClientRpc import (
    ActuatorConfigClient,
)
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.client_stubs.ControlConfigClientRpc import (
    ControlConfigClient,
)
from kortex_api.autogen.client_stubs.DeviceConfigClientRpc import DeviceConfigClient
from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import (
    DeviceManagerClient,
)
from kortex_api.autogen.messages import (
    ActuatorConfig_pb2,
    ActuatorCyclic_pb2,
    Base_pb2,
    BaseCyclic_pb2,
    Common_pb2,
    ControlConfig_pb2,
    DeviceConfig_pb2,
    Session_pb2,
)
from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
from kortex_api.SessionManager import SessionManager
from kortex_api.TCPTransport import TCPTransport
from kortex_api.UDPTransport import UDPTransport


class DeviceConnection:
    IP_ADDRESS = "192.168.1.10"
    TCP_PORT = 10000
    UDP_PORT = 10001

    @staticmethod
    def createTcpConnection():
        return DeviceConnection(port=DeviceConnection.TCP_PORT)

    @staticmethod
    def createUdpConnection():
        return DeviceConnection(port=DeviceConnection.UDP_PORT)

    def __init__(
        self, ip_address=IP_ADDRESS, port=TCP_PORT, credentials=("admin", "admin")
    ):
        self.ip_address = ip_address
        self.port = port
        self.credentials = credentials
        self.session_manager = None
        self.transport = (
            TCPTransport() if port == DeviceConnection.TCP_PORT else UDPTransport()
        )
        self.router = RouterClient(self.transport, RouterClient.basicErrorCallback)

    def __enter__(self):
        self.transport.connect(self.ip_address, self.port)
        if self.credentials[0] != "":
            session_info = Session_pb2.CreateSessionInfo()
            session_info.username = self.credentials[0]
            session_info.password = self.credentials[1]
            session_info.session_inactivity_timeout = 10000  # (milliseconds)
            session_info.connection_inactivity_timeout = 2000  # (milliseconds)
            self.session_manager = SessionManager(self.router)
            print("Logging as", self.credentials[0], "on device", self.ip_address)
            self.session_manager.CreateSession(session_info)
        return self.router

    def __exit__(self, *_):
        if self.session_manager is not None:
            router_options = RouterClientSendOptions()
            router_options.timeout_ms = 1000
            self.session_manager.CloseSession(router_options)
        self.transport.disconnect()


class KinovaArm:
    ACTION_TIMEOUT_DURATION = 60

    def __init__(self):

        # Check whether arm is connected
        try:
            subprocess.run(
                ["ping", "-c", "1", "192.168.1.10"],
                check=True,
                timeout=1,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except subprocess.TimeoutExpired as e:
            raise Exception("Could not communicate with arm") from e

        # Lock file to enforce single instance
        self.lock_file = "/tmp/kinova.lock"
        if os.path.exists(self.lock_file):
            with open(self.lock_file, "r") as f:
                pid = int(f.read().strip())
            try:
                os.kill(pid, 0)
            except OSError:
                print(f"Removing stale lock file (PID {pid})")
                os.remove(self.lock_file)
            else:
                raise Exception(
                    f"Another instance of the arm is already running (PID {pid})"
                )
        with open(self.lock_file, "w") as f:
            f.write(str(os.getpid()))

        # General Kortex setup
        self.tcp_connection = DeviceConnection.createTcpConnection()
        self.udp_connection = DeviceConnection.createUdpConnection()
        self.base = BaseClient(self.tcp_connection.__enter__())
        self.base_cyclic = BaseCyclicClient(self.udp_connection.__enter__())

        self.device_config = DeviceConfigClient(self.base.router)
        self.actuator_config = ActuatorConfigClient(self.base.router)
        self.actuator_count = self.base.GetActuatorCount().count
        self.control_config = ControlConfigClient(self.base.router)
        device_manager = DeviceManagerClient(self.base.router)
        device_handles = device_manager.ReadAllDevices()
        self.actuator_device_ids = [
            handle.device_identifier
            for handle in device_handles.device_handle
            if handle.device_type
            in [Common_pb2.BIG_ACTUATOR, Common_pb2.SMALL_ACTUATOR]
        ]
        self.send_options = RouterClientSendOptions()
        self.send_options.timeout_ms = 3

        # clear faults
        self.clear_faults()

        # Command and feedback setup
        self.base_command = BaseCyclic_pb2.Command()
        for _ in range(self.actuator_count):
            self.base_command.actuators.add()
        self.motor_cmd = self.base_command.interconnect.gripper_command.motor_cmd.add()
        self.base_feedback = BaseCyclic_pb2.Feedback()

        # Make sure actuators are in position mode
        control_mode_message = ActuatorConfig_pb2.ControlModeInformation()
        control_mode_message.control_mode = ActuatorConfig_pb2.ControlMode.Value(
            "POSITION"
        )
        for device_id in self.actuator_device_ids:
            self.actuator_config.SetControlMode(control_mode_message, device_id)

        # Action topic notifications
        self.end_or_abort_event = threading.Event()
        self.end_or_abort_event.set()

        def check_for_end_or_abort(e):
            def check(notification, e=e):
                # print("EVENT : " + Base_pb2.ActionEvent.Name(notification.action_event))
                if notification.action_event in (
                    Base_pb2.ACTION_END,
                    Base_pb2.ACTION_ABORT,
                ):
                    e.set()

            return check

        self.notification_handle = self.base.OnNotificationActionTopic(
            check_for_end_or_abort(self.end_or_abort_event),
            Base_pb2.NotificationOptions(),
        )

    def disconnect(self):
        self.base.Unsubscribe(self.notification_handle)
        self.tcp_connection.__exit__()
        self.udp_connection.__exit__()
        os.remove(self.lock_file)

    def ready(self):
        return self.end_or_abort_event.is_set()

    def wait_ready(self):
        self.end_or_abort_event.wait(KinovaArm.ACTION_TIMEOUT_DURATION)

    def _execute_reference_action(self, action_name, blocking=True):
        # Retrieve reference action
        action_type = Base_pb2.RequestedActionType()
        action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
        action_list = self.base.ReadAllActions(action_type)
        action_handle = None
        for action in action_list.action_list:
            if action.name == action_name:
                action_handle = action.handle
        if action_handle is None:
            return

        # Execute action
        self.end_or_abort_event.clear()
        self.base.ExecuteActionFromReference(action_handle)
        if blocking:
            self.end_or_abort_event.wait(KinovaArm.ACTION_TIMEOUT_DURATION)

    def home(self):
        self._execute_reference_action("Home")

    def retract(self):
        self._execute_reference_action("Retract")

    def zero(self):
        self._execute_reference_action("Zero")

    def get_ee_force(self):
        base_feedback = self.base_cyclic.RefreshFeedback()
        ee_force = np.array(
            [
                base_feedback.base.tool_external_wrench_force_x,
                base_feedback.base.tool_external_wrench_force_y,
                base_feedback.base.tool_external_wrench_force_z,
            ]
        )
        return ee_force

    def get_state(self):

        base_feedback = self.base_cyclic.RefreshFeedback()

        q, dq, tau = (
            np.zeros(self.actuator_count),
            np.zeros(self.actuator_count),
            np.zeros(self.actuator_count),
        )

        ee_pos, ee_vel, ee_force = (
            np.zeros(7),
            np.zeros(7),
            np.zeros(6),
        )

        # Robot joint state
        for i in range(self.actuator_count):
            q[i] = math.radians(base_feedback.actuators[i].position)
            if q[i] > np.pi:
                q[i] -= 2 * np.pi
            dq[i] = math.radians(base_feedback.actuators[i].velocity)
            tau[i] = -base_feedback.actuators[i].torque

        # Robot cartesian state
        ee_pos[:3] = (
            base_feedback.base.tool_pose_x,
            base_feedback.base.tool_pose_y,
            base_feedback.base.tool_pose_z,
        )
        tool_rot = np.array(
            [
                base_feedback.base.tool_pose_theta_x,
                base_feedback.base.tool_pose_theta_y,
                base_feedback.base.tool_pose_theta_z,
            ]
        )
        ee_pos[3:] = R.from_euler("xyz", np.deg2rad(tool_rot)).as_quat()

        ee_vel[:3] = (
            base_feedback.base.tool_twist_linear_x,
            base_feedback.base.tool_twist_linear_y,
            base_feedback.base.tool_twist_linear_z,
        )
        tool_rot_vel = np.array(
            [
                base_feedback.base.tool_twist_angular_x,
                base_feedback.base.tool_twist_angular_y,
                base_feedback.base.tool_twist_angular_z,
            ]
        )
        ee_vel[3:] = R.from_euler("xyz", np.deg2rad(tool_rot_vel)).as_quat()

        ee_force[:3] = (
            base_feedback.base.tool_external_wrench_force_x,
            base_feedback.base.tool_external_wrench_force_y,
            base_feedback.base.tool_external_wrench_force_z,
        )

        gripper_pos = (
            base_feedback.interconnect.gripper_feedback.motor[0].position / 100.0
        )

        return q, ee_pos, gripper_pos
        # return q, dq, tau, ee_pos, ee_vel, ee_force, gripper_pos

    def move_angular_trajectory(self, trajectory_joint_angles, blocking=True):

        assert len(trajectory_joint_angles) > 0, "Invalid trajectory"
        assert (
            len(trajectory_joint_angles[0]) == self.actuator_count
        ), "Invalid number of joint angles"

        jointPoses = [
            [math.degrees(angle) for angle in jointPose]
            for jointPose in trajectory_joint_angles
        ]

        waypoints = Base_pb2.WaypointList()
        waypoints.duration = 0.0
        waypoints.use_optimal_blending = False

        index = 0
        for jointPose in jointPoses:
            waypoint = waypoints.waypoints.add()
            waypoint.name = "waypoint_" + str(index)
            waypoint.angular_waypoint.angles.extend(jointPose)
            waypoint.angular_waypoint.duration = 0.5
            index = index + 1

        result = self.base.ValidateWaypointList(waypoints)
        if len(result.trajectory_error_report.trajectory_error_elements) == 0:
            print("Reaching angular pose trajectory...")

            self.end_or_abort_event.clear()
            self.base.ExecuteWaypointTrajectory(waypoints)

            if blocking:
                print("Waiting for trajectory to finish ...")
                finished = self.end_or_abort_event.wait(
                    KinovaArm.ACTION_TIMEOUT_DURATION
                )
                if finished:
                    print("Angular movement completed")
                else:
                    print("Timeout on action notification wait")
        else:
            print("Error found in trajectory")
            print(result.trajectory_error_report)

    def move_angular(self, joint_angles, blocking=True):

        assert (
            len(joint_angles) == self.actuator_count
        ), "Invalid number of joint angles"

        # Create action
        action = Base_pb2.Action()
        for i in range(self.actuator_count):
            joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
            joint_angle.joint_identifier = i
            joint_angle.value = math.degrees(joint_angles[i])
        self.end_or_abort_event.clear()
        self.base.ExecuteAction(action)
        if blocking:
            print("Waiting for angular movement to finish ...")
            self.end_or_abort_event.wait(KinovaArm.ACTION_TIMEOUT_DURATION)
            # read states and check if the arm actually reached the desired position
            q, _, _ = self.get_state()
            # find error while wrapping angles
            error = np.degrees(q - joint_angles)
            while np.any(error > 180) or np.any(error < -180):
                error = np.where(error > 180, error - 360, error)
                error = np.where(error < -180, error + 360, error)

            if np.any(np.abs(error) > 5):  # 5 degrees
                print("Arm did not reach desired position")
                self.stop()
                print("Arm stopped")
                self.disconnect()
                print("Arm disconnected")
            else:
                print("Angular movement completed")

    def move_cartesian(self, xyz, xyz_quat, blocking=True):

        theta_xyz = R.from_quat(xyz_quat).as_euler("xyz")

        # Create action
        action = Base_pb2.Action()
        cartesian_pose = action.reach_pose.target_pose
        cartesian_pose.x = xyz[0]
        cartesian_pose.y = xyz[1]
        cartesian_pose.z = xyz[2]
        cartesian_pose.theta_x = math.degrees(theta_xyz[0])
        cartesian_pose.theta_y = math.degrees(theta_xyz[1])
        cartesian_pose.theta_z = math.degrees(theta_xyz[2])
        self.end_or_abort_event.clear()
        self.base.ExecuteAction(action)
        if blocking:
            print("Waiting for cartesian movement to finish ...")
            self.end_or_abort_event.wait(KinovaArm.ACTION_TIMEOUT_DURATION)
            # read states and check if the arm actually reached the desired position
            _, x, _ = self.get_state()
            if not np.allclose(x[:3], xyz, atol=0.01):  # 1 cm
                print("Arm did not reach desired position")
                self.stop()
                print("Arm stopped")
                self.disconnect()
                print("Arm disconnected")
            else:
                print("Cartesian movement completed")

    def _gripper_position_command(self, value, blocking=True, timeout=1.0):

        # Send gripper command
        gripper_command = Base_pb2.GripperCommand()
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        finger = gripper_command.gripper.finger.add()
        finger.value = value
        self.base.SendGripperCommand(gripper_command)

        if blocking:
            # Wait for reported position to match value
            gripper_request = Base_pb2.GripperRequest()
            gripper_request.mode = Base_pb2.GRIPPER_POSITION
            start_time = time.perf_counter()
            while time.perf_counter() - start_time < timeout:
                gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
                if abs(value - gripper_measure.finger[0].value) < 0.01:
                    break
                time.sleep(0.01)

    def open_gripper(self, blocking=True):
        self._gripper_position_command(0, blocking)

    def close_gripper(self, blocking=True):
        self._gripper_position_command(1, blocking)

    def set_joint_limits(
        self,
        speed_limits=(60, 60, 60, 60, 60, 60, 60),
        acceleration_limits=(80, 80, 80, 80, 80, 80, 80),
        cartesian=False,
    ):
        if cartesian:
            joint_speed_soft_limits = ControlConfig_pb2.JointSpeedSoftLimits()
            joint_speed_soft_limits.control_mode = (
                ControlConfig_pb2.CARTESIAN_TRAJECTORY
            )
            joint_speed_soft_limits.joint_speed_soft_limits.extend(speed_limits)
            self.control_config.SetJointSpeedSoftLimits(joint_speed_soft_limits)
        else:
            joint_speed_soft_limits = ControlConfig_pb2.JointSpeedSoftLimits()
            joint_speed_soft_limits.control_mode = ControlConfig_pb2.ANGULAR_TRAJECTORY
            joint_speed_soft_limits.joint_speed_soft_limits.extend(speed_limits)
            self.control_config.SetJointSpeedSoftLimits(joint_speed_soft_limits)
            joint_acceleration_soft_limits = (
                ControlConfig_pb2.JointAccelerationSoftLimits()
            )
            joint_acceleration_soft_limits.control_mode = (
                ControlConfig_pb2.ANGULAR_TRAJECTORY
            )
            joint_acceleration_soft_limits.joint_acceleration_soft_limits.extend(
                acceleration_limits
            )
            self.control_config.SetJointAccelerationSoftLimits(
                joint_acceleration_soft_limits
            )

    def set_max_joint_limits(self):
        speed_limits = self.control_config.GetKinematicHardLimits().joint_speed_limits
        acceleration_limits = (
            self.control_config.GetKinematicHardLimits().joint_acceleration_limits
        )
        self.set_joint_limits(speed_limits, acceleration_limits)

    def reset_joint_limits(self):
        control_mode_information = ControlConfig_pb2.ControlModeInformation()
        for control_mode in [
            ControlConfig_pb2.ANGULAR_JOYSTICK,
            ControlConfig_pb2.CARTESIAN_JOYSTICK,
            ControlConfig_pb2.ANGULAR_TRAJECTORY,
            ControlConfig_pb2.CARTESIAN_TRAJECTORY,
            ControlConfig_pb2.CARTESIAN_WAYPOINT_TRAJECTORY,
        ]:
            control_mode_information.control_mode = control_mode
            self.control_config.ResetJointSpeedSoftLimits(control_mode_information)
        for control_mode in [
            ControlConfig_pb2.ANGULAR_JOYSTICK,
            ControlConfig_pb2.ANGULAR_TRAJECTORY,
        ]:
            control_mode_information.control_mode = control_mode
            self.control_config.ResetJointAccelerationSoftLimits(
                control_mode_information
            )

    def set_twist_linear_limit(self, limit):
        twist_linear_soft_limit = ControlConfig_pb2.TwistLinearSoftLimit()
        twist_linear_soft_limit.control_mode = ControlConfig_pb2.CARTESIAN_TRAJECTORY
        twist_linear_soft_limit.twist_linear_soft_limit = limit
        self.control_config.SetTwistLinearSoftLimit(twist_linear_soft_limit)

    def set_max_twist_linear_limit(self):
        limit = self.control_config.GetKinematicHardLimits().twist_linear  # 0.5
        self.set_twist_linear_limit(limit)

    def reset_twist_linear_limit(self):
        control_mode_information = ControlConfig_pb2.ControlModeInformation()
        control_mode_information.control_mode = ControlConfig_pb2.CARTESIAN_TRAJECTORY
        self.control_config.ResetTwistLinearSoftLimit(control_mode_information)

    # Rajat ToDo: Check how the following work:
    def pause_action(self):
        self.base.PauseAction()

    def resume_action(self):
        self.base.ResumeAction()

    def stop_action(self):
        self.base.StopAction()

    def stop(self):
        self.base.Stop()

    def clear_faults(self):
        if self.base.GetArmState().active_state == Base_pb2.ARMSTATE_IN_FAULT:
            self.base.ClearFaults()
            while (
                self.base.GetArmState().active_state != Base_pb2.ARMSTATE_SERVOING_READY
            ):
                time.sleep(0.1)

    def switch_to_gravity_compensation_mode(self):
        raise NotImplementedError

    def switch_to_joint_compliant_mode(
        self,
        command_queue,
        gravity_compensation_external_event,
        gravity_compensation_internal_event,
    ):
        raise NotImplementedError

    def switch_to_task_compliant_mode(
        self,
        command_queue,
        gravity_compensation_external_event,
        gravity_compensation_internal_event,
    ):
        raise NotImplementedError

    def switch_out_of_compliant_mode(self):
        raise NotImplementedError


def main():
    arm = KinovaArm()
    try:
        input("Press Enter to move to retract pos")
        arm.retract()

        input("Press Enter to move to home pos")
        arm.home()

        custom_pos = [
            -2.8655331,
            -1.61973777,
            -2.6097253,
            -1.37301134,
            1.11781087,
            -1.18039928,
            2.05515662,
        ]

        input("Press Enter to move to custom pos (double check DoF to match your robot)")
        arm.move_angular(custom_pos)

    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
