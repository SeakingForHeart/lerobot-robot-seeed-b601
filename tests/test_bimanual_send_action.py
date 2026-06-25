import math
import os

import pytest

from lerobot_robot_seeed_b601 import (
    BiSeeedB601DMFollower,
    BiSeeedB601DMFollowerConfig,
    BiSeeedB601RSFollower,
    BiSeeedB601RSFollowerConfig,
    SeeedB601DMFollower,
    SeeedB601DMFollowerArmConfig,
    SeeedB601DMFollowerConfig,
    SeeedB601RSFollowerArmConfig,
    SeeedB601RSFollowerConfig,
)


@pytest.mark.parametrize(
    ("robot_cls", "bi_config_cls", "arm_config_cls"),
    [
        (BiSeeedB601DMFollower, BiSeeedB601DMFollowerConfig, SeeedB601DMFollowerArmConfig),
        (BiSeeedB601RSFollower, BiSeeedB601RSFollowerConfig, SeeedB601RSFollowerArmConfig),
    ],
)
def test_bimanual_send_action_skips_empty_side(robot_cls, bi_config_cls, arm_config_cls):
    robot = robot_cls(
        bi_config_cls(
            id="regression_dual",
            left_arm_config=arm_config_cls(port="can0", cameras={}),
            right_arm_config=arm_config_cls(port="can1", cameras={}),
        )
    )

    calls = []

    def left_send(action):
        calls.append(("left", dict(action)))
        return dict(action)

    def right_send(action):
        calls.append(("right", dict(action)))
        return dict(action)

    robot.left_arm.send_action = left_send
    robot.right_arm.send_action = right_send

    sent = robot.send_action({"left_shoulder_pan.pos": 12.3})

    assert calls == [("left", {"shoulder_pan.pos": 12.3})]
    assert sent == {"left_shoulder_pan.pos": 12.3}


@pytest.mark.parametrize(
    ("robot_cls", "bi_config_cls", "arm_config_cls"),
    [
        (BiSeeedB601DMFollower, BiSeeedB601DMFollowerConfig, SeeedB601DMFollowerArmConfig),
        (BiSeeedB601RSFollower, BiSeeedB601RSFollowerConfig, SeeedB601RSFollowerArmConfig),
    ],
)
def test_bimanual_send_action_empty_input_sends_nothing(robot_cls, bi_config_cls, arm_config_cls):
    robot = robot_cls(
        bi_config_cls(
            id="regression_dual",
            left_arm_config=arm_config_cls(port="can0", cameras={}),
            right_arm_config=arm_config_cls(port="can1", cameras={}),
        )
    )

    calls = []

    def left_send(action):
        calls.append(("left", dict(action)))
        return dict(action)

    def right_send(action):
        calls.append(("right", dict(action)))
        return dict(action)

    robot.left_arm.send_action = left_send
    robot.right_arm.send_action = right_send

    sent = robot.send_action({})

    assert calls == []
    assert sent == {}


@pytest.mark.parametrize(
    ("robot_cls", "bi_config_cls", "arm_config_cls"),
    [
        (BiSeeedB601DMFollower, BiSeeedB601DMFollowerConfig, SeeedB601DMFollowerArmConfig),
        (BiSeeedB601RSFollower, BiSeeedB601RSFollowerConfig, SeeedB601RSFollowerArmConfig),
    ],
)
def test_bimanual_requires_id(robot_cls, bi_config_cls, arm_config_cls):
    with pytest.raises(ValueError, match="non-empty id"):
        robot_cls(
            bi_config_cls(
                id=None,
                left_arm_config=arm_config_cls(port="can0", cameras={}),
                right_arm_config=arm_config_cls(port="can1", cameras={}),
            )
        )


@pytest.mark.parametrize(
    ("robot_cls", "bi_config_cls", "arm_config_cls"),
    [
        (BiSeeedB601DMFollower, BiSeeedB601DMFollowerConfig, SeeedB601DMFollowerArmConfig),
        (BiSeeedB601RSFollower, BiSeeedB601RSFollowerConfig, SeeedB601RSFollowerArmConfig),
    ],
)
def test_bimanual_requires_distinct_ports(robot_cls, bi_config_cls, arm_config_cls):
    with pytest.raises(ValueError, match="different left and right ports"):
        robot_cls(
            bi_config_cls(
                id="regression_dual",
                left_arm_config=arm_config_cls(port="can0", cameras={}),
                right_arm_config=arm_config_cls(port="can0", cameras={}),
            )
        )


# Hardware-only test for reading and printing both B601 follower arms.
# RS + SocketCAN example:
#   B601_FOLLOWER_TYPE=rs \
#   B601_LEFT_PORT=can1 \
#   B601_RIGHT_PORT=can0 \
#   B601_LEFT_CAN_ADAPTER=socketcan \
#   B601_RIGHT_CAN_ADAPTER=socketcan \
#   PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
#   pytest -q -s tests/test_bimanual_send_action.py::test_bimanual_b601_reads_and_prints_joint_angles
# Optional:
#   B601_DUAL_ID=b601_dual_test
#   B601_DM_SERIAL_BAUD=921600
@pytest.mark.hardware
def test_bimanual_b601_reads_and_prints_joint_angles():
    follower_type = os.environ.get("B601_FOLLOWER_TYPE")
    left_port = os.environ.get("B601_LEFT_PORT")
    right_port = os.environ.get("B601_RIGHT_PORT")
    if not follower_type or not left_port or not right_port:
        pytest.skip("Set B601_FOLLOWER_TYPE, B601_LEFT_PORT, and B601_RIGHT_PORT to run the hardware angle-read test.")
    if follower_type not in {"dm", "rs"}:
        raise ValueError("B601_FOLLOWER_TYPE must be either 'dm' or 'rs'.")

    left_can_adapter = os.environ.get("B601_LEFT_CAN_ADAPTER", "socketcan")
    right_can_adapter = os.environ.get("B601_RIGHT_CAN_ADAPTER", "socketcan")
    dm_serial_baud = int(os.environ.get("B601_DM_SERIAL_BAUD", "921600"))
    follower_id = os.environ.get("B601_DUAL_ID", "b601_dual_test")

    if follower_type == "dm":
        follower = BiSeeedB601DMFollower(
            BiSeeedB601DMFollowerConfig(
                id=follower_id,
                left_arm_config=SeeedB601DMFollowerArmConfig(
                    port=left_port,
                    can_adapter=left_can_adapter,
                    dm_serial_baud=dm_serial_baud,
                    cameras={},
                ),
                right_arm_config=SeeedB601DMFollowerArmConfig(
                    port=right_port,
                    can_adapter=right_can_adapter,
                    dm_serial_baud=dm_serial_baud,
                    cameras={},
                ),
            )
        )
    else:
        follower = BiSeeedB601RSFollower(
            BiSeeedB601RSFollowerConfig(
                id=follower_id,
                left_arm_config=SeeedB601RSFollowerArmConfig(
                    port=left_port,
                    can_adapter=left_can_adapter,
                    cameras={},
                ),
                right_arm_config=SeeedB601RSFollowerArmConfig(
                    port=right_port,
                    can_adapter=right_can_adapter,
                    cameras={},
                ),
            )
        )

    try:
        follower.connect(calibrate=False)
        assert follower.is_connected

        observation = follower.get_observation()
        expected_keys = {
            f"left_{key}" for key in follower.left_arm.action_features
        } | {
            f"right_{key}" for key in follower.right_arm.action_features
        }
        assert set(observation) >= expected_keys

        print("\n[BIMANUAL B601 JOINT ANGLES]")
        for side, arm in (("left", follower.left_arm), ("right", follower.right_arm)):
            arm_obs = arm.get_observation()
            print(f"[{side.upper()}]")
            for motor_name in arm.motor_names:
                angle = arm_obs[f"{motor_name}.pos"]
                assert math.isfinite(angle)
                print(f"  {motor_name:<16} pos={angle:8.2f} deg")
    finally:
        follower.disconnect()


def test_single_arm_send_action_empty_input_returns_empty_action():
    arm = SeeedB601DMFollower(SeeedB601DMFollowerConfig(id="regression_single", port="can0", cameras={}))

    # Simulate a connected state without touching hardware.
    arm.bus = object()

    sent = arm.send_action({})

    assert sent == {}
