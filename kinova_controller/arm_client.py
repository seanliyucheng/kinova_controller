"""
Authors: Jimmy Wu, Rajat Kumar Jenamani
Entrypoint for controlling the robot arm on compute machine.
"""

import threading
import time
import numpy as np

from kinova_controller.arm_interface import (
    ARM_RPC_PORT,
    NUC_HOSTNAME,
    RPC_AUTHKEY,
    ArmInterface,
    ArmManager,
)
from kinova_controller.command_interface import (
    CartesianCommand,
    CloseGripperCommand,
    JointCommand,
    JointTrajectoryCommand,
    KinovaCommand,
    OpenGripperCommand,
)


class ArmInterfaceClient:
    def __init__(self):

        # Register ArmInterface (no lambda needed on the client-side)
        ArmManager.register("ArmInterface")

        # Client setup
        self.manager = ArmManager(
            address=(NUC_HOSTNAME, ARM_RPC_PORT), authkey=RPC_AUTHKEY
        )
        self.manager.connect()

        # This will now use the single, shared instance of ArmInterface
        self._arm_interface = self.manager.ArmInterface()
        self.in_compliant_mode = False

    def switch_to_task_compliant_mode(self):
        assert not self.in_compliant_mode, "Already in compliant mode"
        self._arm_interface.switch_to_task_compliant_mode()
        self.in_compliant_mode = True

    def switch_to_joint_compliant_mode(self):
        assert not self.in_compliant_mode, "Already in compliant mode"
        self._arm_interface.switch_to_joint_compliant_mode()
        self.in_compliant_mode = True

    def switch_out_of_compliant_mode(self):
        assert self.in_compliant_mode, "Not in compliant mode"
        # time.sleep(2.0) # Wait for the arm to settle
        self._arm_interface.switch_out_of_compliant_mode()
        self.in_compliant_mode = False

    def get_state(self):
        return self._arm_interface.get_state()

    def set_tool(self, tool: str):
        assert not self.in_compliant_mode, "Cannot set tool in compliant mode"
        self._arm_interface.set_tool(tool)

    def execute_command(self, cmd: KinovaCommand) -> None:

        # if not self.in_compliant_mode:
        # input("Press enter to execute command...")

        if cmd.__class__.__name__ == "JointTrajectoryCommand":
            return self._arm_interface.set_joint_trajectory(cmd.traj)

        if cmd.__class__.__name__ == "JointCommand":
            if self.in_compliant_mode:
                return self._arm_interface.compliant_set_joint_position(cmd.pos)
            else:
                return self._arm_interface.set_joint_position(cmd.pos)

        if cmd.__class__.__name__ == "CartesianCommand":
            if self.in_compliant_mode:
                return self._arm_interface.compliant_set_ee_pose(cmd.pos, cmd.quat)
            else:
                return self._arm_interface.set_ee_pose(cmd.pos, cmd.quat)

        if cmd.__class__.__name__ == "OpenGripperCommand":
            return self._arm_interface.open_gripper()

        if cmd.__class__.__name__ == "CloseGripperCommand":
            return self._arm_interface.close_gripper()

        raise NotImplementedError(f"Unrecognized command: {cmd}")


if __name__ == "__main__":

    arm_client_interface = ArmInterfaceClient()

    run_commands = input("Press 'y' to run commands")

    if run_commands != "y":
        exit()

    input("Press enter to move to home pos (make sure DoFs in code matches the robot)...")
    home_pos = [
        0, 
        0.26190290154176943, 
        -3.1415700167205274, 
        -2.269006324528677, 
        0, 
        0.959890342232203, 
        1.570794329424079
    ]
    arm_client_interface.execute_command(JointCommand(home_pos))
