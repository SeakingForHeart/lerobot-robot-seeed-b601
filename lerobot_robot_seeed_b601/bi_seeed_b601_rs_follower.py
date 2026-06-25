from functools import cached_property

from lerobot.processor import RobotAction, RobotObservation
from lerobot.robots.robot import Robot

from .config_bi_seeed_b601_rs_follower import BiSeeedB601RSFollowerConfig
from .config_seeed_b601_rs_follower import SeeedB601RSFollowerConfig
from .seeed_b601_rs_follower import SeeedB601RSFollower


class BiSeeedB601RSFollower(Robot):
    """Bimanual wrapper around two Seeed B601 RS follower arms."""

    config_class = BiSeeedB601RSFollowerConfig
    name = "bi_seeed_b601_rs_follower"

    def __init__(self, config: BiSeeedB601RSFollowerConfig):
        if not config.id:
            raise ValueError("Bimanual B601 follower requires a non-empty id.")
        if config.left_arm_config.port == config.right_arm_config.port:
            raise ValueError("Bimanual B601 follower requires different left and right ports.")

        super().__init__(config)
        self.config = config

        left_arm_config = SeeedB601RSFollowerConfig(
            id=f"{config.id}_left" if config.id else None,
            calibration_dir=config.calibration_dir,
            port=config.left_arm_config.port,
            can_adapter=config.left_arm_config.can_adapter,
            dm_serial_baud=config.left_arm_config.dm_serial_baud,
            disable_torque_on_disconnect=config.left_arm_config.disable_torque_on_disconnect,
            max_relative_target=config.left_arm_config.max_relative_target,
            cameras=config.left_arm_config.cameras,
            mit_kp=config.left_arm_config.mit_kp,
            mit_kd=config.left_arm_config.mit_kd,
            motor_can_ids=config.left_arm_config.motor_can_ids,
            joint_limits=config.left_arm_config.joint_limits,
            joint_directions=config.left_arm_config.joint_directions,
            gripper_mit_kp=config.left_arm_config.gripper_mit_kp,
            gripper_mit_kd=config.left_arm_config.gripper_mit_kd,
            gripper_mit_torque_limit=config.left_arm_config.gripper_mit_torque_limit,
            pos_vel_velocity=config.left_arm_config.pos_vel_velocity,
        )

        right_arm_config = SeeedB601RSFollowerConfig(
            id=f"{config.id}_right" if config.id else None,
            calibration_dir=config.calibration_dir,
            port=config.right_arm_config.port,
            can_adapter=config.right_arm_config.can_adapter,
            dm_serial_baud=config.right_arm_config.dm_serial_baud,
            disable_torque_on_disconnect=config.right_arm_config.disable_torque_on_disconnect,
            max_relative_target=config.right_arm_config.max_relative_target,
            cameras=config.right_arm_config.cameras,
            mit_kp=config.right_arm_config.mit_kp,
            mit_kd=config.right_arm_config.mit_kd,
            motor_can_ids=config.right_arm_config.motor_can_ids,
            joint_limits=config.right_arm_config.joint_limits,
            joint_directions=config.right_arm_config.joint_directions,
            gripper_mit_kp=config.right_arm_config.gripper_mit_kp,
            gripper_mit_kd=config.right_arm_config.gripper_mit_kd,
            gripper_mit_torque_limit=config.right_arm_config.gripper_mit_torque_limit,
            pos_vel_velocity=config.right_arm_config.pos_vel_velocity,
        )

        self.left_arm = SeeedB601RSFollower(left_arm_config)
        self.right_arm = SeeedB601RSFollower(right_arm_config)
        self.cameras = {
            **{f"left_{key}": value for key, value in self.left_arm.cameras.items()},
            **{f"right_{key}": value for key, value in self.right_arm.cameras.items()},
        }

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {
            **{f"left_{key}": value for key, value in self.left_arm._motors_ft.items()},
            **{f"right_{key}": value for key, value in self.right_arm._motors_ft.items()},
        }

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            **{f"left_{key}": value for key, value in self.left_arm._cameras_ft.items()},
            **{f"right_{key}": value for key, value in self.right_arm._cameras_ft.items()},
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self.left_arm.is_connected and self.right_arm.is_connected

    def connect(self, calibrate: bool = True) -> None:
        connected_arms = []
        try:
            self.left_arm.connect(calibrate)
            connected_arms.append(self.left_arm)
            self.right_arm.connect(calibrate)
            connected_arms.append(self.right_arm)
        except Exception:
            for arm in reversed(connected_arms):
                try:
                    arm.disconnect()
                except Exception:
                    pass
            raise

    @property
    def is_calibrated(self) -> bool:
        return self.left_arm.is_calibrated and self.right_arm.is_calibrated

    def calibrate(self) -> None:
        print("\n[BIMANUAL CALIBRATION] Calibrating LEFT B601 follower arm.")
        self.left_arm.calibrate()
        print("\n[BIMANUAL CALIBRATION] Calibrating RIGHT B601 follower arm.")
        self.right_arm.calibrate()

    def configure(self) -> None:
        self.left_arm.configure()
        self.right_arm.configure()

    def get_observation(self) -> RobotObservation:
        return {
            **{f"left_{key}": value for key, value in self.left_arm.get_observation().items()},
            **{f"right_{key}": value for key, value in self.right_arm.get_observation().items()},
        }

    def send_action(self, action: RobotAction) -> RobotAction:
        left_action = {
            key.removeprefix("left_"): value for key, value in action.items() if key.startswith("left_")
        }
        right_action = {
            key.removeprefix("right_"): value for key, value in action.items() if key.startswith("right_")
        }

        sent_left = self.left_arm.send_action(left_action) if left_action else {}
        sent_right = self.right_arm.send_action(right_action) if right_action else {}

        return {
            **{f"left_{key}": value for key, value in sent_left.items()},
            **{f"right_{key}": value for key, value in sent_right.items()},
        }

    def disconnect(self) -> None:
        disconnect_errors = []
        for arm in (self.right_arm, self.left_arm):
            if arm.is_connected:
                try:
                    arm.disconnect()
                except Exception as exc:
                    disconnect_errors.append(exc)

        if disconnect_errors:
            raise disconnect_errors[0]
