import time
from platform import system
from pyAgxArm import create_agx_arm_config, AgxArmFactory, ArmModel, PiperFW


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


def create_demo_config():
    platform_system = system()
    if platform_system == "Windows":
        return create_agx_arm_config(
            robot=ArmModel.PIPER_L,
            firmeware_version=PiperFW.DEFAULT,
            interface="agx_cando",
            channel="0",
            # receive_own_messages=True,
            # local_loopback=True,
        )
    if platform_system == "Linux":
        return create_agx_arm_config(
            robot=ArmModel.PIPER_L,
            firmeware_version=PiperFW.DEFAULT,
            interface="socketcan",
            channel="can0",
            # receive_own_messages=True,
            # local_loopback=True,
        )
    if platform_system == "Darwin":
        return create_agx_arm_config(
            robot=ArmModel.PIPER_L,
            firmeware_version=PiperFW.DEFAULT,
            interface="slcan",
            channel="/dev/ttyACM0",
        )
    raise RuntimeError(
        "This demo currently supports Linux `socketcan`, Windows `agx_cando`, and macOS `slcan`."
    )


robot_cfg = create_demo_config()
print(robot_cfg)
robot = AgxArmFactory.create_arm(robot_cfg)
robot.connect()
print(robot.get_channel())
print(robot.__doc__)

end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
# end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.REVO2)
print(end_effector.__doc__)


# -------------------------- Factory ---------------------------

# AgxArmFactory.set_reuse_policy("new")

# robots = []

# for i in range(3):
#     robot = AgxArmFactory.create_arm(robot_cfg)
#     robot.connect()
#     robots.append(robot)

# policy = AgxArmFactory.get_reuse_policy()
# print(policy)

# # When an old instance is no longer referenced,
# # it will be automatically reclaimed by the garbage collector.
# if policy == "new":
#     for robot in robots:
#         print(id(robot), robot.is_connected())

#     # After the list reference is deleted,
#     # the garbage collection mechanism will disconnect the first two instances
#     # and clear the cached items.
#     del robots
#     print(id(robot), robot.is_connected())

# # If the configured fingerprints are the same,
# # the previously created old instance will be returned.
# elif policy == "reuse":
#     for robot in robots:
#         print(id(robot), robot.is_connected())

# # If the configured fingerprints are the same,
# # disconnect the old instance first,
# # then recreate and return the new instance.
# elif policy == "replace":
#     for robot in robots:
#         print(id(robot), robot.is_connected())


# -------------------------- Config ---------------------------

# robot.set_auto_set_motion_mode_enabled(False)
# print(robot.get_auto_set_motion_mode_enabled())
# robot.set_joint_limits_enabled(False)
# print(robot.get_joint_limits_enabled())


# -------------------------- Basic ---------------------------

# while not robot.enable():
#     time.sleep(0.01)

# while not robot.disable():
#     time.sleep(0.01)

# robot.set_speed_percent(100)
# robot.set_installation_pos(robot.OPTIONS.INSTALLATION_POS.HORIZONTAL)
# robot.set_motion_mode(robot.OPTIONS.MOTION_MODE.J)

# print(robot.set_links_vel_acc_period_feedback(False))


# -------------------------- Move ----------------------------

# robot.move_p([0.1, 0.0, 0.3, 0.0, 1.570796326794896619, 0.0])
# wait_motion_done(robot, timeout=5.0)

# robot.move_l([0.2, 0.0, 0.3, 0.0, 1.570796326794896619, 0.0])
# wait_motion_done(robot, timeout=5.0)

# start_pose = [0.2, 0.0, 0.3, 0.0, 1.570796326794896619, 0.0]
# mid_pose = [0.2, 0.05, 0.35, 0.0, 1.570796326794896619, 0.0]
# end_pose = [0.2, 0.0, 0.4, 0.0, 1.570796326794896619, 0.0]
# robot.move_c(start_pose, mid_pose, end_pose)
# wait_motion_done(robot, timeout=5.0)

# robot.move_j([0.0] * robot.joint_nums)
# wait_motion_done(robot, timeout=5.0)


# --------------------------  MIT mode ------------------------

# robot.move_js([0, 0.1, -0.1, 0.0, 0.0, 0.0])
# wait_motion_done(robot, timeout=5.0)

# for i in range(1, robot.joint_nums + 1):
#     robot.move_mit(
#         joint_index=i,
#         p_des=0.0,
#         v_des=0.0,
#         kp=10.0,
#         kd=0.8,
#         t_ff=0.0,
#     )
# wait_motion_done(robot, timeout=5.0)


# -------------------------- reset ----------------------------

# robot.move_j([0] * robot.joint_nums)
# time.sleep(2)

# robot.electronic_emergency_stop()
# time.sleep(1)

# robot.reset()


# -------------------------- Get data -------------------------

while True:
    break

    # print(robot.get_fps())
    # print(robot.is_ok())
    # print(end_effector.get_fps())
    # print(end_effector.is_ok())

    t = time.time()

    # print(robot.get_arm_status())
    # print(end_effector.get_gripper_status())

    # print(robot.get_firmware())
    # print(robot.get_joint_angles())
    # print(robot.get_flange_pose())
    # print(robot.fk([0.0] * robot.joint_nums))

    # print(robot.get_leader_joint_angles())
    # print(end_effector.get_gripper_ctrl_states())

    # print(robot.get_driver_states(1))
    # print(robot.get_motor_states(1))
    # print(robot.get_joint_enable_status(1))
    # print(robot.get_joints_enable_status_list())

    # print(robot.get_flange_vel_acc_limits())

    # print(robot.get_crash_protection_rating())

    # print(end_effector.get_gripper_teaching_pendant_param())

    # for driver_state in [robot.get_driver_states(i) for i in range(1, robot.joint_nums + 1)]:
    #     print(driver_state)
    #     print()

    # for motor_state in [robot.get_motor_states(i) for i in range(1, robot.joint_nums + 1)]:
    #     print(motor_state)
    #     print()

    # for j in [robot.get_joint_angle_vel_limits(i) for i in range(1, robot.joint_nums + 1)]:
    #     print(j)
    #     print()

    # for j in [robot.get_joint_acc_limits(i) for i in range(1, robot.joint_nums + 1)]:
    #     print(j)
    #     print()

    # CPV
    # print([robot.get_cpv_pos(i) for i in range(1, robot.joint_nums + 1)])
    # print([robot.get_cpv_vel(i) for i in range(1, robot.joint_nums + 1)])
    # print([robot.get_cpv_acc(i) for i in range(1, robot.joint_nums + 1)])
    # print([robot.get_cpv_dcc(i) for i in range(1, robot.joint_nums + 1)])
    # print([robot.get_cpv_cv(i) for i in range(1, robot.joint_nums + 1)])
    # print([robot.get_cpv_pp(i) for i in range(1, robot.joint_nums + 1)])
    # print([robot.get_cpv_kp(i) for i in range(1, robot.joint_nums + 1)])
    # print([robot.get_cpv_ki(i) for i in range(1, robot.joint_nums + 1)])

    # Revo2
    # print(end_effector.get_hand_status())
    # print(end_effector.get_finger_pos())
    # print(end_effector.get_finger_spd())
    # print(end_effector.get_finger_current())

    t = round((time.time() - t) * 1000, 4)

    if t > 1:
        print("\nTime elapsed: ", t, "ms")

    print()

    time.sleep(0.005)


# -------------------------- Motor torque measurements -------

# robot.move_mit(6, 0.0, v_des=0.0, kp=0, kd=0, t_ff=0.8)

# import numpy as np

# data = []

# while True:
#     data.append(robot.get_motor_states(6).msg.torque)
#     time.sleep(0.02)
#     if len(data) > 100:
#         break

# data = np.mean(data)
# print(data)


# -------------------------- Gripper --------------------------

# Calibrate the gripper.
# 1. Disable the gripper.
# end_effector.disable_gripper()

# 2. Set the actual maximum stroke of the gripper.
# end_effector.set_gripper_teaching_pendant_param(max_range_config=0.07)  # 0.07m
# end_effector.set_gripper_teaching_pendant_param(max_range_config=0.1)   # 0.1m

# 3. Manually squeeze and close the gripper.
# input("Please squeeze and close the gripper.")

# 4. Run the calibration command.
# end_effector.calibrate_gripper()

# 5. Move the gripper.
# robot.set_motion_mode(robot.OPTIONS.MOTION_MODE.P)
# end_effector.move_gripper_m(0.07)
# time.sleep(0.2)
# end_effector.move_gripper_deg(0)


# -------------------------- Revo2 --------------------------

# end_effector.position_ctrl()
# end_effector.speed_ctrl()
# end_effector.current_ctrl()

# The tip of the thumb moves to position 100 in 2 seconds.
# end_effector.position_time_ctrl(mode='pos', thumb_tip=100)
# end_effector.position_time_ctrl(mode='time', thumb_tip=200)


# -------------------------- Leader-Follower --------------------------

# robot.set_leader_mode()
# robot.set_follower_mode()
# time.sleep(2)
# robot.move_leader_to_home()
# robot.move_leader_follower_to_home()
# time.sleep(2)
# robot.restore_leader_drag_mode()


# ------------------------- Other --------------------------------

# while not robot.disable():
#     time.sleep(0.01)

# print(robot.calibrate_joint(1))
# print(robot.clear_joint_error(1))

# -----------------------------------------------------------------------------------------------

# print(robot.set_payload(robot.OPTIONS.PAYLOAD.EMPTY))

# -----------------------------------------------------------------------------------------------

# print(robot.get_joint_angle_vel_limits(1))
# print(robot.set_joint_angle_vel_limits(joint_index=1, max_joint_spd=3.5))
# print(robot.get_joint_angle_vel_limits(1))


# print(robot.get_joint_acc_limits(1))
# print(robot.set_joint_acc_limits(max_joint_acc=5.0))
# print(robot.get_joint_acc_limits(1))


# print(robot.set_joint_angle_vel_acc_limits_to_default())


# print(robot.get_crash_protection_rating())
# print(robot.set_crash_protection_rating(joint_index=1, rating=1))
# print(robot.get_crash_protection_rating())


# print(robot.get_joint_assistance_rating())
# print(robot.set_joint_assistance_rating(joint_index=1, rating=1))
# print(robot.get_joint_assistance_rating())


# print(end_effector.get_gripper_teaching_pendant_param())
# print(end_effector.set_gripper_teaching_pendant_param(teaching_range_per=100, max_range_config=0.07, teaching_friction=1))
# print(end_effector.get_gripper_teaching_pendant_param())


# print(robot.get_flange_vel_acc_limits())
# print(robot.set_flange_vel_acc_limits(1.0, 0.25, 1.5, 0.3))
# print(robot.set_flange_vel_acc_limits_to_default())
# print(robot.get_flange_vel_acc_limits())


# -------------------------- TCP ----------------------------

# robot.set_tcp_offset([0, 0, 0.1, 0, 0, 0])

# time.sleep(0.1)
# print(robot.get_tcp_pose())

# flange_pose = robot.get_flange_pose()
# if flange_pose is not None:
#     tcp_pose = robot.get_flange2tcp_pose(flange_pose.msg)
#     print(tcp_pose)

# pose = robot.get_tcp2flange_pose([0.3, 0.0, 0.3, 0.0, 1.570796326794896619, 0.0])
# print(pose)
# robot.move_p(pose)
# wait_motion_done(robot, timeout=5.0)


# -------------------------- CPV --------------------------

# robot.move_cpv_pos(1, 0.0)
# robot.move_cpv_vel(1, 0.0)

# ac = robot.get_cpv_acc(1)
# print("ac:", ac)
# print(robot.set_cpv_acc(1, ac))

# dc = robot.get_cpv_dcc(1)
# print("dc:", dc)
# print(robot.set_cpv_dcc(1, dc))

# cv = robot.get_cpv_cv(1)
# print("cv:", cv)
# print(robot.set_cpv_cv(1, cv))

# pp = robot.get_cpv_pp(1)
# print("pp:", pp)
# print(robot.set_cpv_pp(1, pp))

# kp = robot.get_cpv_kp(1)
# print("kp:", kp)
# print(robot.set_cpv_kp(1, kp))

# ki = robot.get_cpv_ki(1)
# print("ki:", ki)
# print(robot.set_cpv_ki(1, ki))

# for i in range(1, robot.joint_nums + 1):
#     robot.set_cpv_cv(i, 1.5)
#     robot.move_cpv_pos(i, 0)
# wait_motion_done(robot, timeout=1)

# for i in range(1, robot.joint_nums + 1):
#     robot.set_cpv_cv(i, 3.0)
#     robot.move_cpv_pos(i, -0.2)
# wait_motion_done(robot, timeout=1)
