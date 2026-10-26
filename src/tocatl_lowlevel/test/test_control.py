"""Offline regression checks for the consolidated robot control interfaces."""
from types import SimpleNamespace as NS

import pytest

from tocatl_lowlevel.motor_commands import JOINT_NAMES, MOTOR_IDS, MockMotors, MotorCommands
from tocatl_lowlevel.homing import ContactConfig, HomingConfig, create_homing
from tocatl_lowlevel.hardware import validate_address_overrides


def batch(stamp=1_000_000_000):
    return NS(header=NS(stamp=NS(sec=stamp // 10**9, nanosec=stamp % 10**9), frame_id=''),
              motors=[NS(motor_id=mid, position_rad=0., velocity_rad_s=0.,
                         torque_nm=0., kp=1., kd=0.1) for mid in MOTOR_IDS])


def test_mock_watchdog_disables_and_requires_rearming():
    now = [0.]
    motors = MockMotors(clock=lambda: now[0])
    motors.enable(True)
    motors.command([JOINT_NAMES[0]], [0.2])
    now[0] = 1.0
    motors.expire()
    assert not motors.enabled and motors.command_timed_out
    with pytest.raises(ValueError, match='disabled'):
        motors.command([JOINT_NAMES[0]], [0.3])


def test_command_validation_is_atomic_and_rejects_replay():
    commands = MotorCommands()
    accepted = commands.accept(batch(), 1_000_000_000)
    bad = batch(1_100_000_000)
    bad.motors[-1].position_rad = float('nan')
    with pytest.raises(ValueError, match='finite'):
        commands.accept(bad, 1_100_000_000)
    assert commands.snapshot()[0] == accepted
    with pytest.raises(ValueError, match='replay'):
        commands.accept(batch(), 1_100_000_000)


def test_feedback_is_required_before_hardware_execution():
    commands = MotorCommands(minimum=[-1.] * 18, maximum=[1.] * 18)
    targets = commands.accept(batch(), 1_000_000_000)
    hardware = NS(snapshot=lambda: ({mid: NS(received=None) for mid in MOTOR_IDS}, ''),
                  stale_after=1.)
    with pytest.raises(ValueError, match='stale'):
        commands.check_feedback(hardware, targets)


def test_can_mapping_rejects_collisions():
    assert validate_address_overrides({12: 27}) == {12: 27}
    with pytest.raises(ValueError, match='collides'):
        validate_address_overrides({12: 13})


@pytest.mark.parametrize('method', ['bench_position_hold', 'bench_oscillation',
                                     'hip_position_hold', 'hip_free_knee_sweep', 'leg1_sweep'])
def test_removed_experiments_cannot_be_started(method):
    node = NS(declare_parameter=lambda name, default: NS(value=method))
    with pytest.raises(ValueError, match='Unsupported homing.method'):
        create_homing(node, None)


def test_homing_configs_still_require_explicit_position_envelopes():
    with pytest.raises(ValueError):
        HomingConfig(motor_id=12, direction=-1, torque_limit=0.1,
                     min_position=0., max_position=0., expected_stop=0.)
    with pytest.raises(ValueError):
        ContactConfig(motor_id=12, minimum=0., maximum=0., expected_stop=0.)


def test_mock_ros_controller_and_state():
    rclpy = pytest.importorskip('rclpy')
    from tocatl_lowlevel.motor_controller import MotorController, build_robot_state
    from sensor_msgs.msg import JointState
    rclpy.init()
    node = None
    try:
        node = MotorController()
        assert node.hardware is None
        node.update()
        state = build_robot_state(node.motors, JointState(), 7, 'base_link')
        assert state.sequence == 7 and state.simulated
        assert len(state.motors) == 18
        assert state.motors_enabled_valid and not state.motors_enabled
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize('method,worker_name', [
    ('torque', 'SingleMotorHoming'),
    ('legacy_impedance', 'ImpedanceContact'),
    ('active_endstop', 'ActiveKneeHoming'),
    ('active_endstop_hip_hold', 'HeldHipKneeHoming'),
])
def test_supported_homing_methods_construct_idle_workers(method, worker_name):
    values = {'homing.method': method, 'homing.motor_id': 13,
              'homing.expected_stop_rad': 0.8, 'homing.min_position_rad': 0.2,
              'homing.max_position_rad': 1.4, 'homing.torque_limit_nm': 0.05,
              'homing.knee_hip_reference_rad': 0.79}
    node = NS(declare_parameter=lambda name, default: NS(value=values.get(name, default)))
    worker = create_homing(node, None)
    assert type(worker).__name__ == worker_name
    assert worker.thread is None and worker.status() == (False, False, 'Homing idle')
    worker.close()
