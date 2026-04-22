import logging
import math
from functools import partial
from typing import Optional


class Validator:
    _LOGGER = logging.getLogger(__name__)

    REF_MAX_ANGLE = 2 * math.pi
    REF_MIN_ANGLE = -REF_MAX_ANGLE

    def __init__(self, logger_: Optional[logging.Logger] = None):
        """Bind a default logger so callers need not pass logger_ repeatedly."""
        self._logger = logger_ if logger_ is not None else self.__class__._LOGGER
        self.clamp_joints = partial(self.__class__.clamp_joints, logger_=self._logger)
        self.clamp_pose6 = partial(self.__class__.clamp_pose6, logger_=self._logger)

    @property
    def logger(self) -> logging.Logger:
        """Expose validator logger for downstream components."""
        return self._logger

    @staticmethod
    def validate_numeric(
        value: float,
        name: str = "value"
    ) -> None:
        """Validate that value is a numeric type and not NaN or Inf.
        """
        if not isinstance(value, (int, float)):
            raise TypeError(f"{name} must be int or float, got {type(value)}")
        if math.isnan(value) or math.isinf(value):
            raise ValueError(f"{name} cannot be NaN or Inf, got {value}")

    @staticmethod
    def validate_list(
        values: list,
        lenth: int,
        name: str = "values"
    ) -> None:
        """Validate that values is a list of given length.
        """
        if not isinstance(values, list):
            raise TypeError(f"{name} must be a list, got {type(values)}")
        if len(values) != lenth:
            raise ValueError(
                f"{name} must have length {lenth}, got {len(values)}"
            )

    @staticmethod
    def validate_min_max(
        min_val: float,
        max_val: float
    ) -> None:
        """Validate that min_val <= max_val.
        """
        if min_val > max_val:
            raise ValueError(
                f"Invalid limit range: min_val ({min_val}) > max_val ({max_val})"
            )

    @staticmethod
    def validate_limits_structure(
        limits: list,
        lenth: int
    ) -> None:
        """Validate that limits is a list of [min, max] pairs.
        """
        Validator.validate_list(limits, lenth, "limits")

        for i, lim in enumerate(limits):
            Validator.validate_list(lim, 2, f"limits[{i}]")
            Validator.validate_min_max(lim[0], lim[1])

    @staticmethod
    def clamp(
        value: float,
        min_val: float,
        max_val: float
    ) -> float:
        """Clamp value within [min_val, max_val].
        """
        return max(min(value, max_val), min_val)

    @staticmethod
    def is_within_limit(
        value: float,
        min_val: float,
        max_val: float,
        tolerance: float = 0.0
    ) -> bool:
        """Check if value is within [min_val - tolerance, max_val + tolerance].
        """
        return (min_val - tolerance) <= value <= (max_val + tolerance)

    @staticmethod
    def is_joints(
        joints: list,
        length: int
    ) -> bool:
        """Validate a list of joint angles in radians.

        - Angle limits: [-2pi, 2pi]
        """
        Validator.validate_list(joints, length, "joints")

        min_val = Validator.REF_MIN_ANGLE
        max_val = Validator.REF_MAX_ANGLE

        for i, j in enumerate(joints):
            Validator.validate_numeric(j, f"joints[{i}]")
            if not Validator.is_within_limit(
                j, min_val, max_val, 0.0
            ):
                return False
        return True

    @staticmethod
    def clamp_joints(
        joints: list,
        length: int,
        joints_limit: list = [],
        logger_: Optional[logging.Logger] = None,
    ) -> list:
        """Clamp joints within given limits.

        - If `joints_limit` is empty, use default limits: [-2pi, 2pi]
        - joints_limit: list of [min, max] pairs for each joint
        """
        Validator.validate_list(joints, length, "joints")

        active_logger = logger_ if logger_ is not None else Validator._LOGGER

        def temp_clamp(i, j, min_val, max_val):
            Validator.validate_numeric(j, f"joints[{i}]")
            if not Validator.is_within_limit(j, min_val, max_val):
                active_logger.warning(
                    "joints[%s] = %s must be within [%s, %s] (unit: rad)",
                    i,
                    j,
                    min_val,
                    max_val,
                )
            return Validator.clamp(j, min_val, max_val)

        clamped = []
        if not joints_limit:
            min_val = Validator.REF_MIN_ANGLE
            max_val = Validator.REF_MAX_ANGLE
            for i, j in enumerate(joints):
                clamped.append(temp_clamp(i, j, min_val, max_val))
        else:
            Validator.validate_limits_structure(
                joints_limit, len(joints)
            )
            for i, (j, (min_val, max_val)) in enumerate(zip(joints, joints_limit)):
                clamped.append(temp_clamp(i, j, min_val, max_val))
        return clamped

    @staticmethod
    def is_pose6(
        pose: list,
        name: str = "pose"
    ) -> bool:
        """Validate a pose6 list: [x, y, z, roll, pitch, yaw].

        - `x/y/z`: meters
        - `roll/pitch/yaw`: radians
            - roll, yaw in `[-pi, pi]`
            - pitch in `[-pi/2, pi/2]`
        """
        Validator.validate_list(pose, 6, name)

        for i, val in enumerate(pose):
            Validator.validate_numeric(val, f"{name}[{i}]")

        if abs(pose[3]) > math.pi:
            return False
        if abs(pose[4]) > (math.pi / 2.0):
            return False
        if abs(pose[5]) > math.pi:
            return False
        return True

    @staticmethod
    def clamp_pose6(
        pose: list,
        name: str = "pose",
        logger_: Optional[logging.Logger] = None,
    ) -> list:
        """Validate a pose6 list: [x, y, z, roll, pitch, yaw].

        - `x/y/z`: meters
        - `roll/pitch/yaw`: radians
            - roll, yaw in `[-pi, pi]`
            - pitch in `[-pi/2, pi/2]`
        """
        Validator.validate_list(pose, 6, name)

        for i, val in enumerate(pose):
            Validator.validate_numeric(val, f"{name}[{i}]")

        active_logger = logger_ if logger_ is not None else Validator._LOGGER

        if abs(pose[3]) > math.pi:
            active_logger.warning(
                "%s[3] = %s must be within [-pi, pi] (unit: rad)",
                name,
                pose[3],
            )
            pose[3] = Validator.clamp(pose[3], -math.pi, math.pi)
        if abs(pose[4]) > (math.pi / 2.0):
            active_logger.warning(
                "%s[4] = %s must be within [-pi/2, pi/2] (unit: rad)",
                name,
                pose[4],
            )
            pose[4] = Validator.clamp(pose[4], -math.pi/2.0, math.pi/2.0)
        if abs(pose[5]) > math.pi:
            active_logger.warning(
                "%s[5] = %s must be within [-pi, pi] (unit: rad)",
                name,
                pose[5],
            )
            pose[5] = Validator.clamp(pose[5], -math.pi, math.pi)

        return pose
