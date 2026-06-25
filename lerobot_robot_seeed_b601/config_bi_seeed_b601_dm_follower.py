from dataclasses import dataclass

from lerobot.robots.robot import RobotConfig

from .config_seeed_b601_dm_follower import SeeedB601DMFollowerArmConfig


@RobotConfig.register_subclass("bi_seeed_b601_dm_follower")
@dataclass
class BiSeeedB601DMFollowerConfig(RobotConfig):
    """Configuration for a pair of Seeed B601 DM follower arms."""

    left_arm_config: SeeedB601DMFollowerArmConfig
    right_arm_config: SeeedB601DMFollowerArmConfig
