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

def move_mit(joints):
    for i, p_des in enumerate(joints):
        if i >= robot.joint_nums:
            break
        robot.move_mit(i+1, p_des=p_des, kp=10, kd=0.8)


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


# -------------------------- Move -----------------------------

# 1.用位置速度模式回到零点
robot.set_motion_mode(robot.MOTION_MODE.J)
robot.move_j([0] * 7)
wait_motion_done(robot, timeout=3.0)


# 2.切换至mit模式，以下的 move_js/mit 二选一即可，效果一样
# 如果move的关节角度跟上一帧的 move_j([0] * 7) 相同，
# motion_status会一直为0，表示已到达目标位置，
# 如果不同，则会一直为1，表示未到达目标位置

# move_js
robot.set_motion_mode(robot.MOTION_MODE.JS)
robot.move_js([0.2] * 7)
wait_motion_done(robot, timeout=2.0)

# move_mit
# robot.set_motion_mode(robot.MOTION_MODE.MIT)
# move_mit([0.2] * 7)
# wait_motion_done(robot, timeout=2.0)


# 3.切换至位置模式，并发送第一帧 move_j/p/l 指令，观察是否被臂实际执行
# 以下的 move_j/p/l 三选一，依次观察情况

# move_j (实际未被臂执行, 如果没有回零，则表示 move_j 指令未被执行)
robot.set_motion_mode(robot.MOTION_MODE.J)
robot.move_j([0] * 7)
wait_motion_done(robot, timeout=3.0)

# move_p (实际会被臂执行)
# robot.set_motion_mode(robot.MOTION_MODE.P)
# robot.move_p([-0.4, -0.0, 0.4, 1.570823, 0.0, 0.0])
# wait_motion_done(robot, timeout=2.0)

# move_l (实际会被臂执行，但是 move_l 在执行过程中，
# 以下的 move_j 指令会被忽略，而 move_js/mit 指令会被执行)
# robot.set_motion_mode(robot.MOTION_MODE.L)
# robot.move_l([-0.4, -0.2, 0.4, 1.570823, 0.0, 0.0])
# robot.set_motion_mode(robot.MOTION_MODE.L)
# wait_motion_done(robot, timeout=5)


# 4.发送第二帧 move_j 指令，才会被臂实际执行
# 观察臂实际的关节运动情况可知，它执行了 move_j 指令
robot.set_motion_mode(robot.MOTION_MODE.J)
robot.move_j([0.0, 0.0, -0.0, 1.2, -0.0, 0.0, 0])
wait_motion_done(robot, timeout=3.0)


# 测试结论：
# mit模式(move_js/mit) -> 位置速度模式(move_j/p/l)：位置速度模式下的第一帧 move_j 指令不会被臂实际执行，但 move_p/l 指令会被执行
