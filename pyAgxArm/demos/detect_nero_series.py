# terminal_monitor.py
# Linux example:   python3 pyAgxArm/demo/detect_arm.py --can_port can0 --hz 10
# Windows example: python3 pyAgxArm/demo/detect_arm.py --can_port 0 --hz 10
import time
import argparse
import os
from platform import system
from pyAgxArm import AgxArmFactory, create_agx_arm_config, NeroFW


def resolve_can_backend():
    platform_system = system()
    if platform_system == "Windows":
        return "agx_cando", "0"
    if platform_system == "Linux":
        return "socketcan", "can0"
    if platform_system == "Darwin":
        return "slcan", "/dev/ttyACM0"
    raise RuntimeError(
        "This demo currently supports Linux `socketcan`, Windows `agx_cando`, and macOS `slcan`."
    )


CAN_INTERFACE, DEFAULT_CAN_PORT = resolve_can_backend()

parser = argparse.ArgumentParser(description="Nero Terminal Table Monitor")
parser.add_argument(
    "--can_port",
    type=str,
    default=DEFAULT_CAN_PORT,
    help="CAN channel. Linux uses names like can0; Windows agx_cando uses indices like 0/1/2.",
)
parser.add_argument("--hz", type=float, default=10, help="Refresh rate (Hz), range: 0.5 ~ 200")
args = parser.parse_args()

exit_flag = False
startup_deadline = time.time() + 15.0

robot_cfg = create_agx_arm_config(
    robot="nero",
    interface=CAN_INTERFACE,
    channel=args.can_port,
)
robot = AgxArmFactory.create_arm(robot_cfg)
robot.connect()

while robot.get_firmware() is None:
    if time.time() >= startup_deadline:
        raise TimeoutError(f"Timed out waiting for firmware on channel {args.can_port}.")
    robot.enable()
    print("Waiting for robot connection...")
    time.sleep(1)

sv = robot.get_firmware()["software_version"]
fw = NeroFW.DEFAULT
if sv >= "1.20":
    fw = NeroFW.V120
elif sv >= "1.12":
    fw = NeroFW.V112
elif sv >= "1.11":
    fw = NeroFW.V111

robot.disconnect()

robot_cfg = create_agx_arm_config(
    robot="nero",
    firmeware_version=fw,
    interface=CAN_INTERFACE,
    channel=args.can_port,
)
robot = AgxArmFactory.create_arm(robot_cfg)
robot.connect()

effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)

print(f"Connected to Nero with firmware {sv}, using config: {robot_cfg}")

def clamp_refresh_rate(rate_hz):
    return max(0.5, min(rate_hz, 200.0))

def clear_terminal():
    os.system("cls" if os.name == "nt" else "clear")

def display_table(refresh_interval):
    global exit_flag
    global args
    start = 0
    while not exit_flag:

        try:
            if time.time() - start > 5.0:
                firmware = robot.get_firmware()
                if firmware is None:
                    time.sleep(0.05)
                    continue

                # joint_angle_vel_limits = [robot.get_joint_angle_vel_limits(i) for i in range(1, robot.joint_nums + 1)]
                # if None in joint_angle_vel_limits:
                #     continue

                # joint_acc_limits = [robot.get_joint_acc_limits(i) for i in range(1, robot.joint_nums + 1)]
                # if None in joint_acc_limits:
                #     continue

                # flange_vel_acc = robot.get_flange_vel_acc_limits()
                # if flange_vel_acc is None:
                #     continue

                # crash_protection = robot.get_crash_protection_rating()
                # if crash_protection is None:
                #     continue

                gripper_teaching_pendant_param = effector.get_gripper_teaching_pendant_param()
                if gripper_teaching_pendant_param is None:
                    time.sleep(0.05)
                    continue

                start = time.time()

            robot_status = robot.get_arm_status()
            if robot_status is None:
                time.sleep(0.05)
                continue

            joint = robot.get_joint_angles()
            if joint is None:
                time.sleep(0.05)
                continue

            flange_pose = robot.get_flange_pose()
            if flange_pose is None:
                time.sleep(0.05)
                continue

            motor_states = [robot.get_motor_states(i) for i in range(1, robot.joint_nums + 1)]
            if None in motor_states:
                time.sleep(0.05)
                continue

            driver_states = [robot.get_driver_states(i) for i in range(1, robot.joint_nums + 1)]
            if None in driver_states:
                time.sleep(0.05)
                continue

            gripper_status = effector.get_gripper_status()
            if gripper_status is None:
                time.sleep(0.05)
                continue

        except Exception as e:
            raise RuntimeError(f"Failed to refresh Nero monitor output: {e}") from e

        clear_terminal()
        print(time.strftime("%a %b %d %H:%M:%S %Y"))
        print(f"+{'='*107}+")
        print(f"Software Ver : {firmware['software_version']:<10}")

        print(f"+{'-'*107}+\n"
              f"{'ArmStatus'} :\n"
              f"{'ctrl_mode':<15}{robot_status.msg.ctrl_mode}\n"
              f"{'arm_status':<15}{robot_status.msg.arm_status}\n"
              f"{'mode_feedback':<15}{robot_status.msg.mode_feedback}\n"
              f"{'motion_status':<15}{robot_status.msg.motion_status}"
              )

        print(f"+{'-'*107}+\n"
              f"|{'JointState':<16}|{'J1':^15}{'J2':^15}{'J3':^15}{'J4':^15}{'J5':^15}{'J6':^15}|\n"
              f"+{'-'*16:^16}+{'-'*90:^}+\n"
              f"|{'position(rad)':<16}|"
              f"{round(joint.msg[0], 3):^15}"
              f"{round(joint.msg[1], 3):^15}"
              f"{round(joint.msg[2], 3):^15}"
              f"{round(joint.msg[3], 3):^15}"
              f"{round(joint.msg[4], 3):^15}"
              f"{round(joint.msg[5], 3):^15}|\n"
              f"|{'position(rad)':<16}|"
              f"{round(motor_states[0].msg.position, 3):^15}"
              f"{round(motor_states[1].msg.position, 3):^15}"
              f"{round(motor_states[2].msg.position, 3):^15}"
              f"{round(motor_states[3].msg.position, 3):^15}"
              f"{round(motor_states[4].msg.position, 3):^15}"
              f"{round(motor_states[5].msg.position, 3):^15}|\n"
              f"|{'cur_spd(rad/s)':<16}|"
              f"{round(motor_states[0].msg.velocity, 3):^15}"
              f"{round(motor_states[1].msg.velocity, 3):^15}"
              f"{round(motor_states[2].msg.velocity, 3):^15}"
              f"{round(motor_states[3].msg.velocity, 3):^15}"
              f"{round(motor_states[4].msg.velocity, 3):^15}"
              f"{round(motor_states[5].msg.velocity, 3):^15}|\n"
              f"|{'current(A)':<16}|"
              f"{round(motor_states[0].msg.current, 3):^15}"
              f"{round(motor_states[1].msg.current, 3):^15}"
              f"{round(motor_states[2].msg.current, 3):^15}"
              f"{round(motor_states[3].msg.current, 3):^15}"
              f"{round(motor_states[4].msg.current, 3):^15}"
              f"{round(motor_states[5].msg.current, 3):^15}|\n"
              f"|{'torque(N.m)':<16}|"
              f"{round(motor_states[0].msg.torque, 3):^15}"
              f"{round(motor_states[1].msg.torque, 3):^15}"
              f"{round(motor_states[2].msg.torque, 3):^15}"
              f"{round(motor_states[3].msg.torque, 3):^15}"
              f"{round(motor_states[4].msg.torque, 3):^15}"
              f"{round(motor_states[5].msg.torque, 3):^15}|\n"
              f"|{'voltage(V)':<16}|"
              f"{round(driver_states[0].msg.vol, 1):^15}"
              f"{round(driver_states[1].msg.vol, 1):^15}"
              f"{round(driver_states[2].msg.vol, 1):^15}"
              f"{round(driver_states[3].msg.vol, 1):^15}"
              f"{round(driver_states[4].msg.vol, 1):^15}"
              f"{round(driver_states[5].msg.vol, 1):^15}|\n"
              f"|{'foc_temp(°C)':<16}|"
              f"{round(driver_states[0].msg.foc_temp):^15}"
              f"{round(driver_states[1].msg.foc_temp):^15}"
              f"{round(driver_states[2].msg.foc_temp):^15}"
              f"{round(driver_states[3].msg.foc_temp):^15}"
              f"{round(driver_states[4].msg.foc_temp):^15}"
              f"{round(driver_states[5].msg.foc_temp):^15}|\n"
              f"|{'motor_temp(°C)':<16}|"
              f"{round(driver_states[0].msg.motor_temp):^15}"
              f"{round(driver_states[1].msg.motor_temp):^15}"
              f"{round(driver_states[2].msg.motor_temp):^15}"
              f"{round(driver_states[3].msg.motor_temp):^15}"
              f"{round(driver_states[4].msg.motor_temp):^15}"
              f"{round(driver_states[5].msg.motor_temp):^15}|\n"
              f"|{'status----------':<16}|{'-'*90:^}|\n"
              f"|{'low_vol_err':<16}|"
              f"{str(driver_states[0].msg.foc_status.voltage_too_low):^15}"
              f"{str(driver_states[1].msg.foc_status.voltage_too_low):^15}"
              f"{str(driver_states[2].msg.foc_status.voltage_too_low):^15}"
              f"{str(driver_states[3].msg.foc_status.voltage_too_low):^15}"
              f"{str(driver_states[4].msg.foc_status.voltage_too_low):^15}"
              f"{str(driver_states[5].msg.foc_status.voltage_too_low):^15}|\n"
              f"|{'motor_overheat':<16}|"
              f"{str(driver_states[0].msg.foc_status.motor_overheating):^15}"
              f"{str(driver_states[1].msg.foc_status.motor_overheating):^15}"
              f"{str(driver_states[2].msg.foc_status.motor_overheating):^15}"
              f"{str(driver_states[3].msg.foc_status.motor_overheating):^15}"
              f"{str(driver_states[4].msg.foc_status.motor_overheating):^15}"
              f"{str(driver_states[5].msg.foc_status.motor_overheating):^15}|\n"
              f"|{'foc_overcurrent':<16}|"
              f"{str(driver_states[0].msg.foc_status.driver_overcurrent):^15}"
              f"{str(driver_states[1].msg.foc_status.driver_overcurrent):^15}"
              f"{str(driver_states[2].msg.foc_status.driver_overcurrent):^15}"
              f"{str(driver_states[3].msg.foc_status.driver_overcurrent):^15}"
              f"{str(driver_states[4].msg.foc_status.driver_overcurrent):^15}"
              f"{str(driver_states[5].msg.foc_status.driver_overcurrent):^15}|\n"
              f"|{'foc_overheat':<16}|"
              f"{str(driver_states[0].msg.foc_status.driver_overcurrent):^15}"
              f"{str(driver_states[1].msg.foc_status.driver_overcurrent):^15}"
              f"{str(driver_states[2].msg.foc_status.driver_overcurrent):^15}"
              f"{str(driver_states[3].msg.foc_status.driver_overcurrent):^15}"
              f"{str(driver_states[4].msg.foc_status.driver_overcurrent):^15}"
              f"{str(driver_states[5].msg.foc_status.driver_overcurrent):^15}|\n"
              f"|{'collision_status':<16}|"
              f"{str(driver_states[0].msg.foc_status.collision_status):^15}"
              f"{str(driver_states[1].msg.foc_status.collision_status):^15}"
              f"{str(driver_states[2].msg.foc_status.collision_status):^15}"
              f"{str(driver_states[3].msg.foc_status.collision_status):^15}"
              f"{str(driver_states[4].msg.foc_status.collision_status):^15}"
              f"{str(driver_states[5].msg.foc_status.collision_status):^15}|\n"
              f"|{'foc_err':<16}|"
              f"{str(driver_states[0].msg.foc_status.driver_error_status):^15}"
              f"{str(driver_states[1].msg.foc_status.driver_error_status):^15}"
              f"{str(driver_states[2].msg.foc_status.driver_error_status):^15}"
              f"{str(driver_states[3].msg.foc_status.driver_error_status):^15}"
              f"{str(driver_states[4].msg.foc_status.driver_error_status):^15}"
              f"{str(driver_states[5].msg.foc_status.driver_error_status):^15}|\n"
              f"|{'enable_status':<16}|"
              f"{str(driver_states[0].msg.foc_status.driver_enable_status):^15}"
              f"{str(driver_states[1].msg.foc_status.driver_enable_status):^15}"
              f"{str(driver_states[2].msg.foc_status.driver_enable_status):^15}"
              f"{str(driver_states[3].msg.foc_status.driver_enable_status):^15}"
              f"{str(driver_states[4].msg.foc_status.driver_enable_status):^15}"
              f"{str(driver_states[5].msg.foc_status.driver_enable_status):^15}|\n"
              f"|{'stall_protection':<16}|"
              f"{str(driver_states[0].msg.foc_status.stall_status):^15}"
              f"{str(driver_states[1].msg.foc_status.stall_status):^15}"
              f"{str(driver_states[2].msg.foc_status.stall_status):^15}"
              f"{str(driver_states[3].msg.foc_status.stall_status):^15}"
              f"{str(driver_states[4].msg.foc_status.stall_status):^15}"
              f"{str(driver_states[5].msg.foc_status.stall_status):^15}|\n"
              f"|{'commuciation_err':<16}|"
              f"{str(robot_status.msg.err_status.communication_status_joint_1):^15}"
              f"{str(robot_status.msg.err_status.communication_status_joint_2):^15}"
              f"{str(robot_status.msg.err_status.communication_status_joint_3):^15}"
              f"{str(robot_status.msg.err_status.communication_status_joint_4):^15}"
              f"{str(robot_status.msg.err_status.communication_status_joint_5):^15}"
              f"{str(robot_status.msg.err_status.communication_status_joint_6):^15}|\n"
              f"|{'over_angle':<16}|"
              f"{str(robot_status.msg.err_status.joint_1_angle_limit):^15}"
              f"{str(robot_status.msg.err_status.joint_2_angle_limit):^15}"
              f"{str(robot_status.msg.err_status.joint_3_angle_limit):^15}"
              f"{str(robot_status.msg.err_status.joint_4_angle_limit):^15}"
              f"{str(robot_status.msg.err_status.joint_5_angle_limit):^15}"
              f"{str(robot_status.msg.err_status.joint_6_angle_limit):^15}|\n"
              f"+{'-'*107}+"
              )
        print(
              f"{'Flange Pose(Euler):':<108}|"
              f"\n"
              f"{'xyz(m)':<12}"
              f"{round(flange_pose.msg[0], 3):<9}"
              f"{round(flange_pose.msg[1], 3):<9}"
              f"{round(flange_pose.msg[2], 3):<9}|"
              f"\n"
              f"{'rpy(rad)':<12}"
              f"{round(flange_pose.msg[3], 3):<9}"
              f"{round(flange_pose.msg[4], 3):<9}"
              f"{round(flange_pose.msg[5], 3):<9}|"
              )
        unit = "m" if gripper_status.msg.mode == "width" else "deg"
        print(f"+{'-'*107}+\n"
              f"{'Gripper&Teaching':<108}|"

              f"\n"
              f"{f'gripper_value({unit})':<21}{round(gripper_status.msg.value, 3):<6}|"
              f"{'Status code :':<59}"
              f"{'|':>22}"
              f"\n"
              f"{'gripper_force(N.m)':<21}{round(gripper_status.msg.force, 3):<6}|"
              f"{'voltage_too_low':<23}{str(gripper_status.msg.foc_status.voltage_too_low):<6}|"
              f"{'motor_overheating':<23}{str(gripper_status.msg.foc_status.motor_overheating):<6}"
              f"{'|':>22}"
              f"\n"
              f"{'teaching_per':<21}{gripper_teaching_pendant_param.msg.teaching_range_per:<6}|"
              f"{'driver_overcurrent':<23}{str(gripper_status.msg.foc_status.driver_overcurrent):<6}|"
              f"{'driver_overheating':<23}{str(gripper_status.msg.foc_status.driver_overheating):<6}"
              f"{'|':>22}"
              f"\n"
              f"{'max_range_config(m)':<21}{gripper_teaching_pendant_param.msg.max_range_config:<6}|"
              f"{'sensor_status':<23}{str(gripper_status.msg.foc_status.sensor_status):<6}|"
              f"{'driver_error_status':<23}{str(gripper_status.msg.foc_status.driver_error_status):<6}"
                f"{'|':>22}"
              f"\n"
              f"{'teaching_friction':<21}{gripper_teaching_pendant_param.msg.teaching_friction:<6}|"
              f"{'driver_enable_status':<23}{str(gripper_status.msg.foc_status.driver_enable_status):<6}|"
              f"{'homing_status':<23}{str(gripper_status.msg.foc_status.homing_status):<6}"
              f"{'|':>22}"
              )
        # 单独打印 FPS 类
        print(f"+{'='*52}FPS{'='*52}+\n"
              f"{'All FPS':<15}: {round(robot.get_fps() + effector.get_fps())}\n"
              f"{'Arm Status':<15}: {round(robot_status.hz):<5}  {'Flange Pose':<15}: {round(flange_pose.hz):<5}\n"
              f"{'Joint Msg':<15}: {round(joint.hz):<5}  {'Gripper Msg':<15}: {round(gripper_status.hz):<5}\n"
              f"{'High Spd Info':<15}: {round(motor_states[0].hz):<5}  {'Low Spd Info':<15}: {round(driver_states[0].hz):<5}")
        print("=" * 109)
        print("Press Ctrl+C to quit")
        time.sleep(refresh_interval)

def main():
    hz = clamp_refresh_rate(args.hz)
    refresh_interval = 1.0 / hz
    try:
        display_table(refresh_interval)
    except KeyboardInterrupt:
        print("\nexit...")
    finally:
        global exit_flag
        exit_flag = True
        robot.disconnect()

if __name__ == "__main__":
    main()
