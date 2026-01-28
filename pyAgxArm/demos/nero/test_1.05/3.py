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

# end_effector = robot.init_effector(robot.EFFECTOR.REVO2)
# print(end_effector.__doc__)

# -------------------------- Basic ---------------------------

robot.set_normal_mode()


# -------------------------- Move -----------------------------

# 1.用mit回到零点
robot.set_motion_mode(robot.MOTION_MODE.JS)
time.sleep(1)
robot.move_js([0, 0, 0, 0, 0, 0, 0])
time.sleep(0.1)
robot.move_js([0, 0, 0, 0, 0, 0, 0])
time.sleep(5)
# 2.检查是否到达
wait_motion_done(robot, timeout=2.0)

robot.set_motion_mode(robot.MOTION_MODE.JS)
# 3.延迟2秒确保mit模式到达
time.sleep(2)
# 4.检查是否到达，到达则打印motion done
status = robot.get_arm_status()
if status is not None and getattr(status.msg, "motion_status", None) == 0:
    print("motion done")
robot.move_js([1, 0, 0, 0, 0, 0, 0])
# 5.延迟2秒确保mit模式到达
time.sleep(2)
# 6.检查是否到达，到达则打印motion done
status = robot.get_arm_status()
if status is not None and getattr(status.msg, "motion_status", None) == 0:
    print("motion done")
robot.move_js([1, 1, 0, 0, 0, 0, 0])
# 7.延迟2秒确保mit模式到达
time.sleep(2)
# 8.检查是否到达，到达则打印motion done
status = robot.get_arm_status()
if status is not None and getattr(status.msg, "motion_status", None) == 0:
    print("motion done")
robot.move_js([1, 1, 1, 0, 0, 0, 0])
# 9.延迟2秒确保mit模式到达
time.sleep(2)
# 10.检查是否到达，到达则打印motion done
status = robot.get_arm_status()
if status is not None and getattr(status.msg, "motion_status", None) == 0:
    print("motion done")
robot.move_js([1, 1, 1, 1, 0, 0, 0])
# 11.延迟2秒确保mit模式到达
time.sleep(2)
# 12.检查是否到达，到达则打印motion done
status = robot.get_arm_status()
if status is not None and getattr(status.msg, "motion_status", None) == 0:
    print("motion done")
robot.move_js([1, 1, 1, 1, 1, 0, 0])
# 13.延迟2秒确保mit模式到达
time.sleep(2)
# 14.检查是否到达，到达则打印motion done
status = robot.get_arm_status()
if status is not None and getattr(status.msg, "motion_status", None) == 0:
    print("motion done")
robot.move_js([1, 1, 1, 1, 1, 0.87, 0])
# 15.延迟2秒确保mit模式到达
time.sleep(2)
# 16.检查是否到达，到达则打印motion done
status = robot.get_arm_status()
if status is not None and getattr(status.msg, "motion_status", None) == 0:
    print("motion done")
robot.move_js([1, 1, 1, 1, 1, 0.87, 1])
# 17.延迟2秒确保mit模式到达
time.sleep(2)
# 18.检查是否到达，到达则打印motion done
status = robot.get_arm_status()
if status is not None and getattr(status.msg, "motion_status", None) == 0:
    print("motion done")
# 19.最后一次检查是否到达
wait_motion_done(robot, timeout=2.0)

# 测试结论：
# 终端不会打印motion done，抓取0x2A1也不反馈到达，web端全程显示到达