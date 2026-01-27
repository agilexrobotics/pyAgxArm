import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory


def wait_motion_done(robot, timeout: float = 5.0, poll_interval: float = 0.1) -> bool:
    """Wait until `robot.get_arm_status().msg.motion_status == 0` or timeout."""
    time.sleep(0.5)
    start_t = time.monotonic()
    while True:
        status = robot.get_arm_status()
        if status is not None and getattr(status.msg, "motion_status", None) == 0:
            print("motion done")
            return True
        if time.monotonic() - start_t > timeout:
            print(f"wait motion done timeout ({timeout:.1f}s)")
            return False
        time.sleep(poll_interval)


robot_cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0", interface="socketcan")
print(robot_cfg)
robot = AgxArmFactory.create_arm(robot_cfg)
robot.connect()
print(robot.get_channel())
print(robot.__doc__)

end_effector = robot.init_effector(robot.EFFECTOR.REVO2)
print(end_effector.__doc__)


# -------------------------- Basic ---------------------------

# robot.set_normal_mode()

# while not robot.enable():
#     time.sleep(0.01)

# while not robot.disable():
#     time.sleep(0.01)

# robot.set_speed_percent(100)
# robot.set_motion_mode(robot.MOTION_MODE.P)


# -------------------------- Move -----------------------------

# The following poses are for Piper and need to be replaced with Nero poses to run successfully on Nero.

# robot.set_motion_mode(robot.MOTION_MODE.P)
# time.sleep(1)
# robot.move_p([-0.4, -0.0, 0.4, 1.570823, 0.0, 0.0])
# time.sleep(0.1)
# robot.set_motion_mode(robot.MOTION_MODE.P)
# wait_motion_done(robot, timeout=2.0)

# robot.set_motion_mode(robot.MOTION_MODE.L)
# robot.move_l([-0.4, -0.2, 0.4, 1.570823, 0.0, 0.0])
# robot.set_motion_mode(robot.MOTION_MODE.L)
# wait_motion_done(robot, timeout=2.5)

# robot.move_l([-0.4, 0.2, 0.4, 1.570823, 0.0, 0.0])
# robot.set_motion_mode(robot.MOTION_MODE.L)
# wait_motion_done(robot, timeout=2.5)

# start_pose = [-0.4, -0.2, 0.4, 1.570823, 0.0, 0.0]
# mid_pose = [-0.4, -0.0, 0.42, 1.570823, 0.0, 0.0]
# end_pose = [-0.4, 0.2, 0.4, 1.570823, 0.0, 0.0]
# robot.set_motion_mode(robot.MOTION_MODE.C)
# time.sleep(0.1)
# robot.move_c(start_pose, mid_pose, end_pose)
# time.sleep(0.1)
# robot.set_motion_mode(robot.MOTION_MODE.C)
# wait_motion_done(robot, timeout=5)

# robot.set_motion_mode(robot.MOTION_MODE.J)
# robot.move_j([0.01] * 7)
# # robot.move_j([0.0, 0.4, -0.0, 1.2, -0.0, 0.0, 0])
# wait_motion_done(robot, timeout=5.0)

# robot.set_motion_mode(robot.MOTION_MODE.JS)
# robot.move_js([0] * 7)


# -------------------------- reset ----------------------------

# robot.move_j([0] * 7)
# time.sleep(2)

# robot.electronic_emergency_stop()
# time.sleep(1)

# robot.reset()


# -------------------------- Get data -------------------------


while True:
    break

    # print(robot.get_fps())
    # print(robot.is_ok())

    t = time.time()

    # print(robot.get_arm_status())

    # print(robot.get_joint_angles())
    # print(robot.get_flange_pose())

    # print(robot.get_driver_states(1))
    # print(robot.get_motor_states(1))
    # print(robot.get_joint_enable_status(1))
    # print(robot.get_joints_enable_status_list())

    t = round((time.time() - t) * 1000, 4)

    if t > 1:
        print("\nTime elapsed: ", t, "ms")

    print()

    time.sleep(0.005)


# -------------------------- Motor torque measurements -------

# robot.move_mit(1, 0.0, v_des=0.0, kp=0, kd=0, t_ff=0.8)

# import numpy as np

# data = []

# while True:
#     data.append(robot.get_motor_states(1).msg.torque)
#     time.sleep(0.02)
#     if len(data) > 100:
#         break

# data = np.mean(data)
# print(data)


# -------------------------- Master Arm --------------------------

# robot.set_master_mode()
# robot.set_slave_mode()


# -------------------------- TCP ----------------------------

# robot.set_tcp_offset([0, 0, 0.1, 0, 0, 0])

# time.sleep(0.1)
# print(robot.get_tcp_pose())

# flange_pose = robot.get_flange_pose()
# if flange_pose is not None:
#     tcp_pose = robot.get_flange2tcp_pose(flange_pose.msg)
#     print(tcp_pose)

# pose = robot.get_tcp2flange_pose([-0.160251, -0.043348, 0.6907249999999999, 1.117935745779928, 0.9272061651219876, 0.14817845349431857])
# robot.move_p(pose)
# time.sleep(2)
# robot.move_p([-0.160251, -0.043348, 0.6907249999999999, 1.117935745779928, 0.9272061651219876, 0.14817845349431857])
