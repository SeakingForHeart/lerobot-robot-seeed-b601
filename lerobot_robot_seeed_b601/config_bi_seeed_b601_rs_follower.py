from dataclasses import dataclass

from lerobot.robots.robot import RobotConfig

from .config_seeed_b601_rs_follower import SeeedB601RSFollowerArmConfig


@RobotConfig.register_subclass("bi_seeed_b601_rs_follower")
@dataclass
class BiSeeedB601RSFollowerConfig(RobotConfig):
    """Configuration for a pair of Seeed B601 RS follower arms."""

    left_arm_config: SeeedB601RSFollowerArmConfig
    right_arm_config: SeeedB601RSFollowerArmConfig
