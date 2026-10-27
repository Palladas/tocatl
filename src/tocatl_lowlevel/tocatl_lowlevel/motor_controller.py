"""ROS motor-space command interface with mock and CyberGear backends."""
import rclpy
from functools import partial
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger
from tocatl_interfaces.msg import RobotState, MotorState, MotorCommand, LegState
from math import nan
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tocatl_lowlevel.motor_commands import MotorCommands, HardwareMotion

from tocatl_lowlevel.motor_commands import JOINT_NAMES, MockMotors
from tocatl_lowlevel.motor_map import MOTORS, hip_knee_ids
from tocatl_lowlevel.homing import LegReadiness


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        backend = self.declare_parameter('backend', 'mock').value
        if backend not in ('mock', 'hardware'):
            raise ValueError('backend must be mock or hardware')
        self.hardware = None
        self.readiness = LegReadiness()
        timeout = self.declare_parameter('command_timeout_s', 1.0).value
        self.motors = MockMotors(timeout)
        self.commands = MotorCommands(
            timeout=timeout,
            max_age=self.declare_parameter('command_max_age_s', 0.5).value,
            minimum=self.declare_parameter('motor_min_position_rad', [0.0] * 18).value,
            maximum=self.declare_parameter('motor_max_position_rad', [0.0] * 18).value,
            max_velocity=self.declare_parameter('max_motor_velocity_rad_s', 0.21).value,
            max_torque=self.declare_parameter('max_motor_torque_nm', 0.5).value,
            max_kp=self.declare_parameter('max_motor_kp', 5.0).value,
            max_kd=self.declare_parameter('max_motor_kd', 1.0).value,
            max_step=self.declare_parameter('max_position_step_rad', 0.1).value,
            motor_ids=self.declare_parameter('actuation_motor_ids', [m.motor_id for m in MOTORS]).value,
            home_offsets={joint: bounds for joint in ('hip_pitch', 'knee')
                          if (bounds := self.declare_parameter(f'{joint}_home_offset_rad', [0.0]).value) != [0.0]},
            homes=lambda: self.readiness.snapshot()[0])
        if backend == 'hardware':
            from tocatl_lowlevel.hardware import HardwareTelemetry
            buses = self.declare_parameter('can_buses', ['can0', 'can1']).value
            if not buses or len(set(buses)) != len(buses) or any(b not in ('can0', 'can1') for b in buses):
                raise ValueError('can_buses must select unique known CAN buses')
            channels = {name: self.declare_parameter(name + '_channel', name).value
                        for name in ('can0', 'can1')}
            self.hardware = HardwareTelemetry(
                lambda: self.get_clock().now().to_msg(),
                {name: channels[name] for name in buses},
                poll_period=self.declare_parameter('poll_period_s', 0.1).value,
                stale_after=self.declare_parameter('telemetry_timeout_s', 1.0).value,
                motor_id_overrides={12: self.declare_parameter('leg1_hip_can_id', 12).value},
                motor_ids=self.declare_parameter(
                    'telemetry_motor_ids', [m.motor_id for m in MOTORS if m.bus in buses]).value)
        torque_limit = self.declare_parameter('motion_torque_limit_nm', 0.0).value
        self.motion = (HardwareMotion(self.hardware, self.commands,
                       self.declare_parameter('allow_hardware_motion', False).value,
                       readiness_check=lambda targets: self.readiness.require(self.hardware, targets),
                       torque_limit=torque_limit or None)
                       if self.hardware is not None else None)
        self.homing = None
        self.homing_was_active = False
        if self.declare_parameter('allow_homing', False).value:
            if self.hardware is None:
                raise ValueError('Homing requires hardware backend')
            from tocatl_lowlevel.homing import create_homing
            self.homing = create_homing(self, self.hardware)
            if hasattr(self.homing, 'readiness'):
                self.readiness = self.homing.readiness
            elif self.motion.allowed:
                raise ValueError('Combined homing/actuation requires homing.method=hip_knee')
            self.homing_service = self.create_service(Trigger, 'motors/home', self.home)
            self.leg_homing_services = [
                self.create_service(Trigger, f'motors/leg_{leg}/home', partial(self.home, leg_id=leg))
                for leg in getattr(self.homing, 'leg_ids', ())]
        self.motor_subscription = self.create_subscription(
            MotorCommand, 'motor_command', self.motor_command,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE))
        self.sequence = 0
        self.body_frame = self.declare_parameter('body_frame', 'base_link').value
        self.state_publisher = self.create_publisher(RobotState, 'robot_state', 10)
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.subscription = self.create_subscription(
            JointState, 'joint_commands', self.command, 10)
        self.service = self.create_service(SetBool, 'motors/enable', self.enable)
        self.timer = self.create_timer(0.02, self.update)
        self.get_logger().info(
            ('Hardware telemetry ready; motor commands require explicit arming'
             if self.motion.allowed else 'Hardware telemetry ready')
            if self.hardware else 'Mock motors ready, disabled. No CAN devices opened.')

    def enable(self, request, response):
        if self.hardware is not None:
            if self.homing is not None and (not hasattr(self.homing, 'readiness') or self.homing.active):
                if request.data:
                    response.success = False
                    response.message = 'Normal arming unavailable in homing mode; use motors/home'
                else:
                    self.motion.enable(False)
                    self.homing.cancel()
                    response.success = True
                    response.message = 'Homing cancellation requested; inspect robot_state for stop status'
                return response
            try:
                self.motion.enable(request.data)
                response.success = True
                response.message = 'Hardware arm requested' if request.data else 'Hardware disarm requested'
            except ValueError as error:
                response.success = False
                response.message = str(error)
            return response
        self.motors.enable(request.data)
        targets, age = self.commands.snapshot()
        if request.data and targets and age < self.commands.timeout:
            self.apply_mock_targets(targets)
        if not request.data:
            self.commands.clear()
        response.success = True
        response.message = 'Mock motors enabled' if request.data else 'Mock motors disabled'
        return response

    def home(self, request, response, leg_id=None):
        try:
            with self.motion.lock:
                if self.motion.requested or self.motion.armed or self.motion.stop_pending:
                    raise ValueError('Disarm and confirm stopped motors before homing')
            self.commands.clear()
            if leg_id is None:
                self.homing.start()
            else:
                self.homing.start([leg_id])
            self.homing_was_active = True
            selection = f'leg {leg_id}' if leg_id is not None else 'configured legs'
            self.get_logger().info(f'Homing started for {selection}')
            response.success, response.message = True, 'Homing requested'
        except ValueError as error:
            response.success, response.message = False, str(error)
        return response

    def apply_mock_targets(self, targets):
        # Instant motor position tracking; no IK or dynamic simulation.
        by_id = {m.motor_id: name for m, name in zip(
            MOTORS, JOINT_NAMES)}
        self.motors.positions.update({by_id[t.motor_id]: t.position_rad for t in targets})
        self.motors.command_received = True
        self.motors.command_timed_out = False
        self.motors.last_command = self.commands.received

    def motor_command(self, message):
        try:
            check = (lambda targets: self.commands.check_feedback(self.hardware, targets)
                     ) if self.motion is not None and self.motion.allowed else None
            targets = self.commands.accept(message, self.get_clock().now().nanoseconds, check)
            if self.hardware is None and self.motors.enabled:
                self.apply_mock_targets(targets)
        except ValueError as error:
            self.get_logger().warning(f'motor_command rejected: {error}')

    def command(self, message):
        if self.hardware is not None:
            self.get_logger().warning('Use motor_command for hardware motor-space targets')
            return
        try:
            if message.velocity or message.effort:
                raise ValueError('Only position commands are supported')
            self.motors.command(message.name, message.position)
        except ValueError as error:
            self.get_logger().warning(str(error))

    def update(self):
        if self.hardware is not None:
            if self.homing is not None:
                active = self.homing.active
                if self.homing_was_active and not active:
                    _, failed, message = self.homing.status()
                    # Separate call sites: rclpy rejects one call site logging at two severities.
                    if failed:
                        self.get_logger().error(message)
                    else:
                        self.get_logger().info(message)
                self.homing_was_active = active
            state = build_hardware_state(self.hardware, self.get_clock().now().to_msg(),
                                         self.sequence, self.body_frame)
            armed, faulted, reason = self.motion.status()
            if self.homing is not None and (self.homing.active or self.homing.status()[1] or
                                            not hasattr(self.homing, 'readiness')) and not armed:
                armed, faulted, reason = self.homing.status()
            state.safety_state = RobotState.FAULT if faulted else (RobotState.ARMED if armed else RobotState.DISARMED)
            transport_reason = state.status_reason
            state.status_reason = reason
            if transport_reason != 'Read-only hardware telemetry; arming unavailable':
                state.status_reason += '; ' + transport_reason
            self.add_leg_status(state)
            if (self.homing is not None and not self.homing.active and
                    state.safety_state == RobotState.DISARMED and any(l.ready for l in state.legs)):
                state.status_reason = self.homing.status()[2]
            self.add_command_status(state)
            self.state_publisher.publish(state)
            self.sequence += 1
            return
        self.motors.expire()
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(JOINT_NAMES)
        message.position = [self.motors.positions[name] for name in JOINT_NAMES]
        self.publisher.publish(message)
        state = build_robot_state(
            self.motors, message, self.sequence, self.body_frame)
        self.add_leg_status(state)
        if self.commands.snapshot()[1] is not None:
            self.add_command_status(state)
        self.state_publisher.publish(state)
        self.sequence += 1

    def add_leg_status(self, state):
        homing = getattr(self, 'homing', None)
        motion = getattr(self, 'motion', None)
        homing_fault = homing is not None and homing.status()[1]
        motion_fault = motion is not None and motion.status()[1]
        if state.safety_state == RobotState.FAULT and (motion_fault or not homing_fault):
            self.readiness.invalidate(f'Controller fault: {state.status_reason}; rehome required')
        ready = self.hardware is not None and self.readiness.check(self.hardware)
        homes, _ = self.readiness.snapshot()
        for leg_id in range(1, 7):
            leg = LegState()
            leg.leg_id, leg.scope = leg_id, 'pitch_knee'
            leg.motor_ids = list(hip_knee_ids(leg_id))
            leg.homed = all(mid in homes for mid in leg.motor_ids)
            leg.ready = bool(ready and leg.homed)
            leg.home_positions_rad = [homes[mid] for mid in leg.motor_ids] if leg.homed else []
            leg.status_reason = self.readiness.leg_reason(leg_id)
            state.legs.append(leg)

    def add_command_status(self, state):
        from math import nan
        targets, age = self.commands.snapshot()
        state.accepted_motor_command_stamp.sec = self.commands.last_stamp // 1_000_000_000
        state.accepted_motor_command_stamp.nanosec = self.commands.last_stamp % 1_000_000_000
        state.command_received = age is not None
        state.command_age_s = nan if age is None else age
        state.command_timeout_s = self.commands.timeout
        state.command_timed_out = age is not None and age >= self.commands.timeout

    def destroy_node(self):
        if self.homing is not None:
            self.homing.close()
        if self.motion is not None:
            self.motion.close()
        if self.hardware is not None:
            self.hardware.close()
        return super().destroy_node()


def build_robot_state(motors, joints, sequence, body_frame):
    state = RobotState()
    state.header.stamp = joints.header.stamp
    state.header.frame_id = body_frame
    state.sequence = sequence
    state.backend = 'mock'
    state.simulated = True
    state.safety_state = RobotState.DISARMED
    state.status_reason = ('Mock command watchdog expired' if motors.command_timed_out
                           else 'Mock backend; physical safety readiness is unavailable')
    state.motors_enabled_valid = True
    state.motors_enabled = motors.enabled
    state.command_received = motors.command_received
    state.command_age_s = (max(0.0, motors.clock() - motors.last_command)
                           if motors.command_received else nan)
    state.command_timeout_s = motors.timeout_s
    state.command_timed_out = motors.command_timed_out
    state.telemetry_fresh = True
    state.joints = joints
    for name, motor in zip(JOINT_NAMES, MOTORS):
        sample = MotorState()
        sample.header = state.header
        sample.joint_name = name
        sample.motor_id = motor.motor_id
        sample.bus = motor.bus
        sample.telemetry_valid = True
        sample.telemetry_fresh = True
        sample.enabled_valid = True
        sample.enabled = motors.enabled
        sample.position_rad = motors.positions[name]
        sample.velocity_rad_s = nan
        sample.torque_estimate_nm = nan
        sample.temperature_c = nan
        sample.bus_voltage_v = nan
        sample.mode = 'mock_position'
        state.motors.append(sample)
    return state


def build_hardware_state(hardware, stamp, sequence, body_frame):
    import time
    from sensor_msgs.msg import JointState
    samples, error = hardware.snapshot()
    now = time.monotonic()
    state = RobotState()
    state.header.stamp = stamp
    state.header.frame_id = body_frame
    state.sequence = sequence
    state.backend = 'hardware'
    state.safety_state = RobotState.DISARMED
    state.status_reason = error or 'Read-only hardware telemetry; arming unavailable'
    state.command_age_s = nan
    state.command_timeout_s = nan
    state.joints = JointState()
    state.joints.header = state.header
    for name, motor in zip(JOINT_NAMES, MOTORS):
        raw = samples[motor.motor_id]
        sample = MotorState()
        sample.joint_name, sample.motor_id, sample.bus = name, motor.motor_id, motor.bus
        if raw.stamp is not None:
            sample.header.stamp = raw.stamp
        # Raw encoder coordinates: no joint calibration transform is applied.
        sample.telemetry_valid = raw.received is not None
        sample.telemetry_fresh = (raw.received is not None and
                                  now - raw.received < hardware.stale_after)
        sample.fault_status_valid = (raw.fault_received is not None and
                                     now - raw.fault_received < hardware.stale_after)
        sample.position_rad, sample.velocity_rad_s = raw.position, raw.velocity
        sample.torque_estimate_nm, sample.temperature_c = raw.torque, raw.temperature
        sample.bus_voltage_v = nan
        sample.mode = '' if raw.mode is None else str(raw.mode)
        sample.enabled_valid = sample.telemetry_fresh and raw.mode in (0, 1, 2)
        sample.enabled = sample.telemetry_fresh and raw.mode == 2
        sample.faults = sorted(set(raw.faults + raw.state_faults))
        state.motors.append(sample)
    state.telemetry_fresh = all(m.telemetry_fresh for m in state.motors)
    state.fault_status_valid = all(m.fault_status_valid for m in state.motors)
    state.motors_enabled_valid = all(m.enabled_valid for m in state.motors)
    state.motors_enabled = any(m.enabled for m in state.motors)
    state.faults = [f'{m.joint_name}: {fault}' for m in state.motors for fault in m.faults]
    # Raw motor angles are not calibrated robot joint angles; leave joints empty.
    return state


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MotorController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.motors.enable(False)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
