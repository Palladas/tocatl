"""Drive tocatl_walk from an Xbox controller (sensor_msgs/Joy from joy_node).

| Input | Action |
|---|---|
| Left stick | walk in any direction (/cmd_vel linear x, y) |
| Right stick left/right | turn (/cmd_vel angular z) |
| Right trigger | speed: 30% when released, 100% fully pressed |
| **LB held** | **deadman**: the robot only walks while LB is held |
| Start held 1 s | arm (/motors/enable true): tocatl_walk rises, then walks |
| B | stop: walk/stop, which settles the feet, lowers the body and disarms |
| X | next gait: ripple, then tetrapod, then tripod (walk/gait; applied once the feet settle) |
| D-pad up/down | body height up/down 1 cm (walk/height; among heights tocatl_walk verified) |
| Back or Xbox button | DISARM now (/motors/enable false), the hard stop; a standing robot drops |
| **LB held** + A / Y / RB / left-stick click / right-stick click | feet-in-air cue (perform/cue, for tocatl_perform): wave / bloom / circle / alternate / wave_reverse |

Cues are only sent while LB is held, so a stray press does nothing. tocatl_walk ignores them,
and tocatl_perform ignores /cmd_vel: run one of the two, never both.

Velocity is only non-zero while LB is held and /joy is fresh. If no /joy arrives for
JOY_TIMEOUT_S (pad out of range or unplugged), the node publishes zero, so the walker
settles. Maximum speeds are tocatl_walk's; the walker clamps them again, and teleop never
widens a limit. Button and axis numbers can be
changed with ROS parameters. Defaults were recorded from /joy (joy_node, SDL2) on 2026-11-07 with
an Xbox Series controller over Bluetooth (xpadneo, 9 axes, 11 buttons). Over USB (xpad, 8 axes) the
D-pad up/down is axis 7: pass ``-p axis_dpad_y:=7``.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import sys
import time

GAIT_CYCLE = ('ripple', 'tetrapod', 'tripod')
# PadLayout button field -> tocatl_perform cue, sent only while LB (deadman) is held.
CUE_BUTTONS = (('button_a', 'wave'), ('button_y', 'bloom'), ('button_rb', 'circle'),
               ('button_ls', 'alternate'), ('button_rs', 'wave_reverse'))
JOY_TIMEOUT_S = 0.25
ARM_HOLD_S = 1.0
PUBLISH_PERIOD_S = 0.05


@dataclass(frozen=True)
class PadLayout:
    """Indices in sensor_msgs/Joy, recorded over Bluetooth (xpadneo) on 2026-11-07."""
    axis_lx: int = 0
    axis_ly: int = 1
    axis_rx: int = 3
    axis_rt: int = 5          # 1.0 released .. -1.0 pressed
    axis_dpad_y: int = 8      # +1 up, -1 down (7 over USB/xpad)
    button_a: int = 0
    button_b: int = 1
    button_x: int = 2
    button_y: int = 3
    button_lb: int = 4
    button_rb: int = 5
    button_back: int = 6
    button_start: int = 7
    button_xbox: int = 8
    button_ls: int = 9        # left stick click
    button_rs: int = 10       # right stick click


@dataclass(frozen=True)
class TeleopLimits:
    max_vx_m_s: float = 0.2
    max_vy_m_s: float = 0.2
    max_wz_rad_s: float = 0.2
    deadzone: float = 0.12
    expo: float = 0.5           # 0 = linear, 1 = cubic
    min_speed_scale: float = 0.3


def shape(value, deadzone, expo):
    """Deadzone, rescale to [-1, 1], then blend linear and cubic for fine control near centre."""
    magnitude = abs(value)
    if magnitude <= deadzone:
        return 0.0
    x = min(1.0, (magnitude - deadzone) / (1.0 - deadzone))
    return (1 if value > 0 else -1) * ((1 - expo) * x + expo * x ** 3)


@dataclass
class TeleopMapper:
    """Pure joystick logic: feed ``update(axes, buttons, now)`` per /joy; read the outputs."""
    layout: PadLayout = PadLayout()
    limits: TeleopLimits = TeleopLimits()
    gait_index: int = 0
    last_joy: float = None
    previous: tuple = ()
    previous_dpad: float = 0.0
    start_since: float = None
    armed_this_hold: bool = False
    events: list = field(default_factory=list)   # 'arm', 'disarm', 'stop', ('gait'|'height'|'cue', value)
    twist: tuple = (0.0, 0.0, 0.0)

    def _button(self, buttons, index):
        return index < len(buttons) and bool(buttons[index])

    def _axis(self, axes, index, default=0.0):
        return float(axes[index]) if index < len(axes) else default

    def update(self, axes, buttons, now):
        lay, lim = self.layout, self.limits
        pressed = lambda i: self._button(buttons, i) and not (i < len(self.previous) and self.previous[i])
        self.last_joy = now
        if pressed(lay.button_back) or pressed(lay.button_xbox):
            self.events.append('disarm')
        if pressed(lay.button_b):
            self.events.append('stop')
        #if pressed(lay.button_x):
            #self.gait_index = (self.gait_index + 1) % len(GAIT_CYCLE)
            #self.events.append(('gait', GAIT_CYCLE[self.gait_index]))
        if self._button(buttons, lay.button_start):
            self.start_since = now if self.start_since is None else self.start_since
            if not self.armed_this_hold and now - self.start_since >= ARM_HOLD_S:
                self.events.append('arm')
                self.armed_this_hold = True
        else:
            self.start_since, self.armed_this_hold = None, False
        if self._button(buttons, lay.button_lb):
            for button, cue in CUE_BUTTONS:
                if pressed(getattr(lay, button)):
                    self.events.append(('cue', cue))
        self.previous = tuple(buttons)
        dpad = self._axis(axes, lay.axis_dpad_y)
        step = 1 if dpad > 0.5 else -1 if dpad < -0.5 else 0
        if step and step != (1 if self.previous_dpad > 0.5 else -1 if self.previous_dpad < -0.5 else 0):
            self.events.append(('height', step))
        self.previous_dpad = dpad
        if not self._button(buttons, lay.button_lb):
            self.twist = (0.0, 0.0, 0.0)
            return
        trigger = self._axis(axes, lay.axis_rt, 1.0)
        scale = lim.min_speed_scale + (1 - lim.min_speed_scale) * min(1.0, max(0.0, (1.0 - trigger) / 2))
        self.twist = (scale * lim.max_vx_m_s * shape(self._axis(axes, lay.axis_ly), lim.deadzone, lim.expo),
                      scale * lim.max_vy_m_s * shape(self._axis(axes, lay.axis_lx), lim.deadzone, lim.expo),
                      scale * lim.max_wz_rad_s * shape(self._axis(axes, lay.axis_rx), lim.deadzone, lim.expo))

    def command(self, now):
        """The twist to publish now: zero if /joy has gone quiet."""
        if self.last_joy is None or now - self.last_joy > JOY_TIMEOUT_S:
            return (0.0, 0.0, 0.0)
        return self.twist

    def take_events(self):
        events, self.events = self.events, []
        return events


def main(argv=None):
    import rclpy
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Joy
    from std_msgs.msg import Empty, Int8, String
    from std_srvs.srv import SetBool
    rclpy.init(args=argv)
    node = rclpy.create_node('tocatl_teleop')
    layout = PadLayout(**{name: node.declare_parameter(name, getattr(PadLayout(), name)).value
                          for name in PadLayout.__dataclass_fields__})
    limits = TeleopLimits(**{name: float(node.declare_parameter(name, getattr(TeleopLimits(), name)).value)
                             for name in TeleopLimits.__dataclass_fields__})
    mapper = TeleopMapper(layout=layout, limits=limits)
    cmd_vel = node.create_publisher(Twist, 'cmd_vel', 1)
    stop = node.create_publisher(Empty, 'walk/stop', 1)
    gait = node.create_publisher(String, 'walk/gait', 1)
    height = node.create_publisher(Int8, 'walk/height', 1)
    cue = node.create_publisher(String, 'perform/cue', 1)
    enable = node.create_client(SetBool, 'motors/enable')
    node.create_subscription(Joy, 'joy', lambda m: mapper.update(m.axes, m.buttons, time.monotonic()), 10)

    def call_enable(value):
        if not enable.service_is_ready():
            node.get_logger().error('motors/enable unavailable; is motor_controller running?')
            return
        future = enable.call_async(SetBool.Request(data=value))
        future.add_done_callback(lambda f: node.get_logger().info(
            f'{"ARM" if value else "DISARM"}: {f.result().message if f.result() else "no reply"}'))

    def tick():
        for event in mapper.take_events():
            if event == 'disarm':
                call_enable(False)
            elif event == 'arm':
                call_enable(True)
            elif event == 'stop':
                stop.publish(Empty())
                node.get_logger().info('Stop: settling and sitting')
            elif event[0] == 'cue':
                cue.publish(String(data=event[1]))
                node.get_logger().info(f'Cue {event[1]}')
            elif event[0] == 'height':
                height.publish(Int8(data=event[1]))
                node.get_logger().info(f'Body height {"up" if event[1] > 0 else "down"} (once the feet settle)')
            else:
                gait.publish(String(data=event[1]))
                node.get_logger().info(f'Gait {event[1]} (applies once the feet settle)')
        vx, vy, wz = mapper.command(time.monotonic())
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        cmd_vel.publish(msg)
    node.create_timer(PUBLISH_PERIOD_S, tick)
    node.get_logger().info('Teleop ready: hold LB to walk or to fire cues (A/Y/RB/stick clicks), Start 1 s to arm, '
                           'B to sit, Back/Xbox to disarm')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cmd_vel.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
