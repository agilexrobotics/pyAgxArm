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

# 设置普通模式后，第一帧move指令才能得以被执行，
# 且会开启外网can推送，使能部分才能获取到关节使能状态，
# 才能通过以下的使能循环
robot.set_normal_mode()

while not robot.enable():
    time.sleep(0.01)


# -------------------------- case1 -----------------------------

robot.set_motion_mode(robot.MOTION_MODE.P)
robot.move_p([-0.4, -0.0, 0.4, 1.570823, 0.0, 0.0])
robot.set_motion_mode(robot.MOTION_MODE.P)
# 如果超时，则表示未到达目标位置，但实际臂已经运动完成了
wait_motion_done(robot, timeout=2.0)

robot.set_motion_mode(robot.MOTION_MODE.L)
robot.move_l([-0.4, -0.2, 0.4, 1.570823, 0.0, 0.0])
robot.set_motion_mode(robot.MOTION_MODE.L)
# 如果超时，则表示未到达目标位置，但实际臂已经运动完成了
wait_motion_done(robot, timeout=3.0)

robot.move_l([-0.4, 0.2, 0.4, 1.570823, 0.0, 0.0])
robot.set_motion_mode(robot.MOTION_MODE.L)
# 如果超时，则表示未到达目标位置，但实际臂已经运动完成了
wait_motion_done(robot, timeout=3.0)

# motion_status: REACH_TARGET_POS_FAILED(0x1) 表示未到达目标位置
# 相当于 `candump can0 | grep 2A1` 中的 Byte[4]
print(robot.get_arm_status().msg.motion_status)


# -------------------------- case2 -----------------------------

# 1.先用关节运动，捕获一帧 pose 数据，如果没有成功捕获，则退出程序
robot.set_motion_mode(robot.MOTION_MODE.J)
robot.move_j([10 / 180 * 3.14159] * 7)
wait_motion_done(robot, timeout=3.0)
pose = robot.get_flange_pose()
if pose is None:
    print("get flange pose failed")
    exit(1)
else:
    pose = pose.msg
    print(f"flange pose: {pose}")

# 2.回零
robot.move_j([0] * 7)
time.sleep(2)
wait_motion_done(robot, timeout=3.0)

# 3.切换到笛卡尔运动，用捕获的 pose 作为目标
robot.set_motion_mode(robot.MOTION_MODE.P)
robot.move_p(pose)
# 如果超时，则表示未到达目标位置，但实际臂已经运动完成了
wait_motion_done(robot, timeout=3.0)


# 测试结论：
# move_j 可以正常反馈是否到达目标位置
# move_p/l 不能正常反馈是否到达目标位置，但实际上已经到达目标位置
