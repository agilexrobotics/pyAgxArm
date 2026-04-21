# ---------- robot 可选字段 ----------

ROBOT_OPTION_FIELDS = {
    "nero": {
        "joint_limits",
        "auto_set_motion_mode",
        "enable_joint_limits",
    },
    "piper": {
        "joint_limits",
        "auto_set_motion_mode",
        "enable_joint_limits",
    },
    "piper_h": {
        "joint_limits",
        "auto_set_motion_mode",
        "enable_joint_limits",
    },
    "piper_l": {
        "joint_limits",
        "auto_set_motion_mode",
        "enable_joint_limits",
    },
    "piper_x": {
        "joint_limits",
        "auto_set_motion_mode",
        "enable_joint_limits",
    },
}

# ---------- 预定义机械臂关节名字 ----------

ROBOT_JOINT_NAME = {
    "nero": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"],
    "piper": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
    "piper_h": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
    "piper_l": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
    "piper_x": ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
}

# ---------- 预定义机械臂关节限位 ----------

ROBOT_JOINT_LIMIT_PRESET_RAD = {
    "nero": {
        "joint1": [-2.705260, 2.705260],
        "joint2": [-1.740000, 1.740000],
        "joint3": [-2.750000, 2.750000],
        "joint4": [-1.010000, 2.140000],
        "joint5": [-2.750000, 2.750000],
        "joint6": [-0.730000, 0.950000],
        "joint7": [-1.5707963, 1.5707963],
    },
    "piper": {
        "joint1": [-2.6179938, 2.6179938],
        "joint2": [0.0, 3.1415926],
        "joint3": [-2.9670597, 0.0],
        "joint4": [-1.7453292, 1.7453292],
        "joint5": [-1.2217304, 1.2217304],
        "joint6": [-2.0943951, 2.0943951],
    },
    "piper_h": {
        "joint1": [-2.6179938, 2.6179938],
        "joint2": [0.0, 3.1415926],
        "joint3": [-2.9670597, 0.0],
        "joint4": [-2.3561944, 2.3561944],
        "joint5": [-1.5620696, 1.5620696],
        "joint6": [-2.0943951, 2.0943951],
    },
    "piper_l": {
        "joint1": [-2.6179938, 2.6179938],
        "joint2": [0.0, 3.1415926],
        "joint3": [-2.9670597, 0.0],
        "joint4": [-2.2165681, 2.2165681],
        "joint5": [-1.5620696, 1.5620696],
        "joint6": [-2.0943951, 2.0943951],
    },
    "piper_x": {
        "joint1": [-2.6179938, 2.6179938],
        "joint2": [0.0, 3.1415926],
        "joint3": [-2.9670597, 0.0],
        "joint4": [-1.5533430, 1.5533430],
        "joint5": [-1.5533430, 1.5533430],
        "joint6": [-2.0943951, 2.0943951],
    },
}

ROBOT_JOINT_LIMIT_PRESET_DEG = {
    "nero": {
        "joint1": [-155.0, 155.0],
        "joint2": [-99.7, 99.7],
        "joint3": [-157.6, 157.6],
        "joint4": [-57.9, 122.6],
        "joint5": [-157.6, 157.6],
        "joint6": [-41.8, 54.4],
        "joint7": [-90.0, 90.0],
    },
    "piper": {
        "joint1": [-150.0, 150.0],
        "joint2": [0.0, 180.0],
        "joint3": [-170.0, 0.0],
        "joint4": [-100.0, 100.0],
        "joint5": [-70.0, 70.0],
        "joint6": [-120.0, 120.0],
    },
    "piper_h": {
        "joint1": [-150.0, 150.0],
        "joint2": [0.0, 180.0],
        "joint3": [-170.0, 0.0],
        "joint4": [-135.0, 135.0],
        "joint5": [-89.5, 89.5],
        "joint6": [-120.0, 120.0],
    },
    "piper_l": {
        "joint1": [-150.0, 150.0],
        "joint2": [0.0, 180.0],
        "joint3": [-170.0, 0.0],
        "joint4": [-127.0, 127.0],
        "joint5": [-89.5, 89.5],
        "joint6": [-120.0, 120.0],
    },
    "piper_x": {
        "joint1": [-150.0, 150.0],
        "joint2": [0.0, 180.0],
        "joint3": [-170.0, 0.0],
        "joint4": [-89.0, 89.0],
        "joint5": [-89.0, 89.0],
        "joint6": [-120.0, 120.0],
    },
}

ROBOT_JOINT_LIMIT_PRESET = ROBOT_JOINT_LIMIT_PRESET_RAD
