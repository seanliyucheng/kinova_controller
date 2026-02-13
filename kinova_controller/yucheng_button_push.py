from kinova_controller.kinova import KinovaArm
# x=0.4928 y=0.0076 z=1.0679 qx=0.000000 qy=0.000000 qz=0.000000 qw=1.000000
# x=0.8362 y=-0.1118 z=0.4745 qx=0.000000 qy=0.000000 qz=0.000000 qw=1.000000

xyz = [0.8362, -0.1118, 0.4745]
quat = [0.5, 0.5, 0.5, 0.5]

retract_xyz = [0.12, 0.0, 0.31]
retract_quat = [0.7, 0.7, 0.0, 0.0] # identity orientation (x, y, z, w)

home_xyz = [0.45, 0.0, 0.45]
home_quat = [0.5, 0.5, 0.5, 0.5]

def main():
    arm = KinovaArm()
    try:
        arm.set_max_joint_limits()
        arm.set_max_twist_linear_limit()
        # arm.move_cartesian(xyz, quat, blocking=True)

        arm.move_cartesian(home_xyz, home_quat, blocking=True)
    finally:
        arm.disconnect()

if __name__ == "__main__":
    main()
