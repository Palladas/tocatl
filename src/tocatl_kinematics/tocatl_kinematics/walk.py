"""Online omnidirectional walking: gait phase, foot placement, whole-body IK and checks.

TOCATL's six legs sit every 60 degrees around the body (leg 1 at 30 degrees, numbered in
order), so there is no front: any heading and any turn rate are the same gait.

The command is a body twist (vx, vy, wz) in base_link. A foot on the ground is fixed in
the world, so in the body frame it moves by -(v + w x p). A swinging foot lifts on a
cycloid and lands at a Raibert-style target, its neutral position plus half the stance
displacement ahead, so the next stance is centred on neutral. The gait sets which legs
swing together (phase offsets) and how long each foot is down (duty):

  tripod    duty 0.55  legs 1/3/5 then 2/4/6 (fastest; three legs carry the body)
  tetrapod  duty 0.70  opposite pairs 1+4, 2+5, 3+6 swing in turn
  ripple    duty 0.85  one leg at a time, order 1 4 2 5 3 6 (slowest; five legs carry)

Duties a little above 1 - 1/groups leave a short all-feet-down overlap at each handover.
Commands are clamped to the stride the stance time allows and rate-limited. With a zero
command the gait keeps stepping until every foot is back at neutral, then stops.

Offline, ``simulate`` runs a command script and ``check`` reports joint limits, reach,
static stability (centre of mass inside the stance polygon), per-motor torque for the
robot's weight shared by the stance feet, and motor speeds. Nothing here touches CAN.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from math import atan2, cos, hypot, pi, sin

import numpy as np

from tocatl_kinematics.body import body_to_leg, neutral_stance
from tocatl_kinematics.geometry import JOINTS, LEG_IDS, RobotGeometry
from tocatl_kinematics.joint_map import JointMap, motor_ids
from tocatl_kinematics.leg import KinematicsError, ik, jacobian

GRAVITY = 9.81
RATED_TORQUE_NM = 4.0      # CyberGear continuous
CAP_MARGIN = 0.9           # of the controller's motion torque cap
SETTLED_M = 0.002          # feet this close to neutral count as settled
COMMAND_EPS = 1e-4

GAITS = {
    'tripod': (0.55, {1: 0.0, 3: 0.0, 5: 0.0, 2: 0.5, 4: 0.5, 6: 0.5}),
    'tetrapod': (0.70, {1: 0.0, 4: 0.0, 2: 1 / 3, 5: 1 / 3, 3: 2 / 3, 6: 2 / 3}),
    'ripple': (0.85, {1: 0.0, 4: 1 / 6, 2: 2 / 6, 5: 3 / 6, 3: 4 / 6, 6: 5 / 6}),
}


@dataclass(frozen=True)
class WalkParams:
    # Defaults (2026-11-13, Pranav: faster, more yaw travel accepted): a 0.2 m/s tripod. Checked
    # offline at 16 kg in every direction: peak ~10 Nm, hip-pitch motor ~6.0 rad/s (needs the
    # 6.5 rad/s max_motor_velocity_rad_s in motion-standing.yaml), yaw ~0.43 rad. Tetrapod and
    # ripple need a longer period_s and lower speeds.
    gait: str = 'tripod'
    foot_radius_m: float = 0.25     # foot distance from each yaw axis (stand uses 0.25)
    body_height_m: float = 0.15     # feet this far below the mounting plane (stand height)
    step_height_m: float = 0.08     # swing lift; 0.04 dragged the feet (2026-11-07, tripod)
    period_s: float = 1.5           # one full gait cycle
    max_stride_m: float = 0.2       # largest foot travel during one stance
    max_vx_m_s: float = 0.2         # (vx, vy) is clamped to the ellipse these define
    max_vy_m_s: float = 0.2
    max_wz_rad_s: float = 0.2
    # Command ramp. 0.05 / 0.2 took 1-2 s to change direction (2026-11-07, Pranav: too slow);
    # these reach full speed in ~0.1-0.25 s.
    accel_m_s2: float = 0.4
    alpha_rad_s2: float = 1.6

    def validate(self):
        problems = []
        if self.gait not in GAITS:
            problems.append(f'gait must be one of {sorted(GAITS)}')
        if not 0.1 <= self.foot_radius_m <= 0.6:
            problems.append('foot_radius_m must be in [0.1, 0.6]')
        if not 0.05 <= self.body_height_m <= 0.5:
            problems.append('body_height_m must be in [0.05, 0.5]')
        if not 0.005 <= self.step_height_m <= 0.2:
            problems.append('step_height_m must be in [0.005, 0.2]')
        if not 0.5 <= self.period_s <= 20:
            problems.append('period_s must be in [0.5, 20]')
        if not 0 < self.max_stride_m <= 1:
            problems.append('max_stride_m must be in (0, 1]')
        if not all(0 <= v <= 1 for v in (self.max_vx_m_s, self.max_vy_m_s)) or not 0 <= self.max_wz_rad_s <= 1.0:
            problems.append('speed limits must be within 1 m/s and 1 rad/s')
        if not (0 < self.accel_m_s2 <= 1.0 and 0 < self.alpha_rad_s2 <= 2.0):
            problems.append('accelerations must be positive and modest')
        if problems:
            raise ValueError('Invalid walk: ' + '; '.join(problems))
        return self

    @property
    def duty(self):
        return GAITS[self.gait][0]

    @property
    def stance_s(self):
        return self.duty * self.period_s


def _cycloid(u):
    return u - np.sin(2 * pi * u) / (2 * pi)


def _foot_velocity(cmd, xy):
    vx, vy, wz = cmd
    return np.array([vx - wz * xy[1], vy + wz * xy[0]])


@dataclass
class WalkGenerator:
    """Stateful gait: call ``step(dt, command)`` each tick; ``feet`` are body-frame targets."""
    geometry: RobotGeometry
    params: WalkParams
    legs: tuple = LEG_IDS
    phase: float = 0.0
    command: tuple = (0.0, 0.0, 0.0)          # filtered body twist being executed
    feet: dict = field(default_factory=dict)
    neutral: dict = field(default_factory=dict)
    swinging: dict = field(default_factory=dict)  # leg -> lift-off position

    def __post_init__(self):
        self.params.validate()
        self.neutral = neutral_stance(self.geometry, self.params.foot_radius_m, self.params.body_height_m, self.legs)
        if not self.feet:
            self.feet = {leg: p.copy() for leg, p in self.neutral.items()}
        self.duty, self.offsets = GAITS[self.params.gait]

    def leg_phase(self, leg, phase=None):
        return ((self.phase if phase is None else phase) + self.offsets[leg]) % 1.0

    def in_swing(self, leg, phase=None):
        return self.leg_phase(leg, phase) >= self.duty

    def contact(self):
        return {leg: not self.in_swing(leg) for leg in self.legs}

    def clamp(self, command):
        """Limit a requested twist to the speed limits and the stride the stance allows."""
        p = self.params
        vx, vy, wz = (float(c) for c in command)
        # Elliptical, not per-axis: a diagonal command must not exceed what feasibility() checks
        # (it walks the ellipse); per-axis clamping let a corner run sqrt(2) faster.
        vx = vx if p.max_vx_m_s > 0 else 0.0
        vy = vy if p.max_vy_m_s > 0 else 0.0
        norm = hypot(vx / p.max_vx_m_s if vx else 0.0, vy / p.max_vy_m_s if vy else 0.0)
        if norm > 1.0:
            vx, vy = vx / norm, vy / norm
        wz = max(-p.max_wz_rad_s, min(p.max_wz_rad_s, wz))
        worst = max(hypot(*_foot_velocity((vx, vy, wz), n[:2])) for n in self.neutral.values())
        scale = min(1.0, p.max_stride_m / (worst * p.stance_s)) if worst > 0 else 1.0
        return vx * scale, vy * scale, wz * scale

    def _ramp(self, target, dt):
        p = self.params
        limits = (p.accel_m_s2 * dt, p.accel_m_s2 * dt, p.alpha_rad_s2 * dt)
        return tuple(c + max(-l, min(l, t - c)) for c, t, l in zip(self.command, target, limits))

    def settled(self):
        return not any(self.in_swing(leg) for leg in self.legs) and all(
            np.linalg.norm(self.feet[leg] - self.neutral[leg]) <= SETTLED_M for leg in self.legs)

    def idle(self):
        return max(abs(c) for c in self.command) < COMMAND_EPS and self.settled()

    def step(self, dt, requested=(0.0, 0.0, 0.0)):
        """Advance ``dt`` seconds toward the ``requested`` twist; returns the executed twist."""
        target = self.clamp(requested)
        self.command = self._ramp(target, dt)
        if max(abs(c) for c in target) < COMMAND_EPS and self.idle():
            self.command = (0.0, 0.0, 0.0)
            return self.command
        p = self.params
        self.phase = (self.phase + dt / p.period_s) % 1.0
        for leg in self.legs:
            foot, neutral = self.feet[leg], self.neutral[leg]
            ground = neutral[2]
            if not self.in_swing(leg):
                if leg in self.swinging:  # touched down this tick
                    del self.swinging[leg]
                xy = foot[:2] - _foot_velocity(self.command, foot[:2]) * dt
                self.feet[leg] = np.array([xy[0], xy[1], ground])
                continue
            if leg not in self.swinging:
                self.swinging[leg] = foot.copy()
            u = (self.leg_phase(leg) - self.duty) / (1 - self.duty)
            lift = self.swinging[leg]
            landing = neutral[:2] + _foot_velocity(self.command, neutral[:2]) * p.stance_s / 2
            xy = lift[:2] + (landing - lift[:2]) * _cycloid(u)
            z = ground + p.step_height_m * (1 - cos(2 * pi * u)) / 2
            self.feet[leg] = np.array([xy[0], xy[1], z])
        return self.command


def solve(geometry: RobotGeometry, feet: dict, branch='knee_down'):
    """Body-frame feet -> {leg: IKSolution}; joint limits are checked separately."""
    return {leg: ik(body_to_leg(p, geometry.leg(leg)), geometry.leg(leg), branch, check_limits=False)
            for leg, p in feet.items()}


def support_forces(feet_xy: dict, mass_kg: float, com_xy=(0.0, 0.0)):
    """Least-norm vertical foot forces (N) holding ``mass_kg`` with its centre of mass at
    ``com_xy``: sum F = m g and the moments about the CoM balance. Negative values mean the
    CoM is outside the stance polygon (a foot would have to pull)."""
    legs = list(feet_xy)
    if not legs:
        return {}
    a = np.vstack([np.ones(len(legs)), [feet_xy[l][0] - com_xy[0] for l in legs],
                   [feet_xy[l][1] - com_xy[1] for l in legs]])
    b = np.array([mass_kg * GRAVITY, 0.0, 0.0])
    forces = np.linalg.pinv(a) @ b
    return dict(zip(legs, forces))


def stability_margin(feet_xy: dict, com_xy=(0.0, 0.0)):
    """Signed distance (m) from the CoM to the nearest edge of the stance support polygon:
    positive inside. Fewer than three feet gives -inf."""
    points = [np.asarray(p[:2], float) for p in feet_xy.values()]
    if len(points) < 3:
        return float('-inf')
    centre = np.mean(points, axis=0)
    hull = sorted(points, key=lambda q: atan2(q[1] - centre[1], q[0] - centre[0]))
    com = np.asarray(com_xy, float)
    margin = float('inf')
    for i, a in enumerate(hull):
        b = hull[(i + 1) % len(hull)]
        edge = b - a
        length = np.linalg.norm(edge)
        if length < 1e-9:
            continue
        # counter-clockwise hull: inside is to the left of each edge
        margin = min(margin, float((edge[0] * (com[1] - a[1]) - edge[1] * (com[0] - a[0])) / length))
    return margin


@dataclass
class WalkTrajectory:
    params: WalkParams
    times: np.ndarray
    feet: dict          # leg -> (n, 3) body-frame
    contact: dict       # leg -> (n,) bool
    q: dict             # leg -> (n, 3) joint angles
    command: np.ndarray  # (n, 3) executed twist
    body: np.ndarray    # (n, 3) world x, y, heading from the executed twist
    failure: str = ''


def simulate(geometry: RobotGeometry, params: WalkParams, script, dt=0.02, legs=LEG_IDS, settle_s=None):
    """Run ``script`` [(seconds, (vx, vy, wz)), ...], then a zero command until settled.

    The first unreachable foot stops the simulation and is reported in ``failure``."""
    gen = WalkGenerator(geometry, params, tuple(legs))
    times, feet, contact, q, command, body = [], {l: [] for l in legs}, {l: [] for l in legs}, \
        {l: [] for l in legs}, [], []
    pose = np.zeros(3)
    t, failure = 0.0, ''
    settle_s = 3 * params.period_s if settle_s is None else settle_s
    schedule = [(d, c) for d, c in script] + [(settle_s, (0.0, 0.0, 0.0))]

    def record():
        nonlocal failure
        try:
            solutions = solve(geometry, gen.feet)
        except KinematicsError as error:
            failure = f't={t:.2f} s: {error}'
            return False
        times.append(t)
        for leg in legs:
            feet[leg].append(gen.feet[leg].copy())
            contact[leg].append(not gen.in_swing(leg))
            q[leg].append(solutions[leg].q)
        command.append(gen.command)
        body.append(pose.copy())
        return True

    record()
    for duration, cmd in schedule:
        for _ in range(int(round(duration / dt))):
            executed = gen.step(dt, cmd)
            c, s = cos(pose[2]), sin(pose[2])
            pose += np.array([c * executed[0] - s * executed[1], s * executed[0] + c * executed[1], executed[2]]) * dt
            t += dt
            if not record():
                break
        if failure:
            break
    return WalkTrajectory(params=params, times=np.array(times), feet={l: np.array(v) for l, v in feet.items()},
                          contact={l: np.array(v) for l, v in contact.items()},
                          q={l: np.array(v) for l, v in q.items()}, command=np.array(command),
                          body=np.array(body), failure=failure)


def motor_velocities(joint_map: JointMap, leg: int, q: np.ndarray, dt: float):
    """Raw (motor) rad/s for each of the leg's three motors, from joint angles over time."""
    qd = np.gradient(q, dt, axis=0) if len(q) > 1 else np.zeros_like(q)
    yaw, hip, knee = (joint_map.motors[m] for m in motor_ids(leg))
    return np.column_stack([yaw.sign * yaw.ratio * qd[:, 0], hip.sign * hip.ratio * qd[:, 1],
                            knee.sign * knee.ratio * qd[:, 2] + knee.coupling_raw_per_hip_rad * qd[:, 1]])


def check(trajectory: WalkTrajectory, geometry: RobotGeometry, joint_map: JointMap, mass_kg: float,
          torque_limit_nm: float = 12.0, max_motor_speed_rad_s: float = 5.0, min_margin_m: float = 0.03,
          min_reach_m: float = 0.01, yaw_envelope=(-0.3, 0.3)):
    """Feasibility report for a simulated walk; ``ok`` only if there are no violations."""
    violations, warnings = [], []
    if trajectory.failure:
        violations.append(f'Unreachable: {trajectory.failure}')
    if len(trajectory.times) < 2:
        return dict(ok=False, violations=violations or ['empty trajectory'], warnings=warnings)
    dt = float(trajectory.times[1] - trajectory.times[0])
    legs = list(trajectory.q)
    n = len(trajectory.times)
    peak_torque, rms_torque, peak_speed = {}, {}, {}
    torques = {m: np.zeros(n) for leg in legs for m in motor_ids(leg)}
    margins = np.full(n, np.inf)
    for i in range(n):
        stance = {leg: trajectory.feet[leg][i] for leg in legs if trajectory.contact[leg][i]}
        margins[i] = stability_margin(stance)
        for leg, force in support_forces(stance, mass_kg).items():
            joint = jacobian(trajectory.q[leg][i], geometry.leg(leg)).T @ np.array([0.0, 0.0, force])
            for k, motor_id in enumerate(motor_ids(leg)):
                torques[motor_id][i] = joint[k] / joint_map.motors[motor_id].ratio
    worst_margin = float(np.min(margins))
    if worst_margin < min_margin_m:
        at = float(trajectory.times[int(np.argmin(margins))])
        violations.append(f'centre of mass comes within {worst_margin * 1000:.0f} mm of the stance polygon edge '
                          f'at t={at:.2f} s (need {min_margin_m * 1000:.0f} mm)')
    for leg in legs:
        q = trajectory.q[leg]
        for k, (name, (low, high)) in enumerate(zip(JOINTS, geometry.leg(leg).limits.as_tuple())):
            if q[:, k].min() < low or q[:, k].max() > high:
                violations.append(f'leg {leg} {name} leaves [{low:.2f}, {high:.2f}]: {q[:, k].min():.3f}..{q[:, k].max():.3f}')
        yaw_offset = q[:, 0] - joint_map.motors[motor_ids(leg)[0]].q_at_reference_rad
        if yaw_offset.min() < yaw_envelope[0] or yaw_offset.max() > yaw_envelope[1]:
            violations.append(f'leg {leg} yaw needs {yaw_offset.min():+.3f}..{yaw_offset.max():+.3f} rad from radial, '
                              f'outside the controller envelope {yaw_envelope}')
        reach = min(ik(body_to_leg(p, geometry.leg(leg)), geometry.leg(leg), check_limits=False).reach_margin_m
                    for p in trajectory.feet[leg][::5])
        if reach < min_reach_m:
            violations.append(f'leg {leg} comes within {reach * 1000:.0f} mm of its reach limit')
        speeds = np.abs(motor_velocities(joint_map, leg, q, dt))
        for k, motor_id in enumerate(motor_ids(leg)):
            peak_speed[motor_id] = float(speeds[:, k].max())
            if peak_speed[motor_id] > max_motor_speed_rad_s:
                violations.append(f'motor {motor_id} peaks at {peak_speed[motor_id]:.2f} rad/s > {max_motor_speed_rad_s}')
            column = torques[motor_id]
            peak_torque[motor_id] = float(np.abs(column).max())
            rms_torque[motor_id] = float(np.sqrt(np.mean(column ** 2)))
            if peak_torque[motor_id] > CAP_MARGIN * torque_limit_nm:
                violations.append(f'motor {motor_id} needs ~{peak_torque[motor_id]:.1f} Nm, over {CAP_MARGIN:.0%} of '
                                  f'the {torque_limit_nm:g} Nm cap')
            elif rms_torque[motor_id] > RATED_TORQUE_NM:
                warnings.append(f'motor {motor_id} averages ~{rms_torque[motor_id]:.1f} Nm RMS, above the '
                                f'{RATED_TORQUE_NM:g} Nm continuous rating: watch temperature')
    travel = trajectory.body[-1]
    return dict(ok=not violations, violations=violations, warnings=warnings,
                gait=trajectory.params.gait, duration_s=float(trajectory.times[-1]),
                min_stability_margin_m=worst_margin,
                peak_torque_nm=max(peak_torque.values()), peak_torque_by_motor=peak_torque,
                rms_torque_by_motor=rms_torque, peak_motor_speed_rad_s=max(peak_speed.values()),
                body_travel=dict(x_m=float(travel[0]), y_m=float(travel[1]), heading_rad=float(travel[2])))


FEASIBILITY_DIRECTIONS = 8


def feasibility(geometry: RobotGeometry, joint_map: JointMap, params: WalkParams, mass_kg: float, **limits):
    """Walk at the maximum command in eight directions and at the maximum turn rate, each for
    two cycles; the worst case decides whether these parameters are safe to stream."""
    cycles = 2 * params.period_s
    scripts = [[(cycles, (params.max_vx_m_s * cos(a), params.max_vy_m_s * sin(a), 0.0))]
               for a in np.linspace(0, 2 * pi, FEASIBILITY_DIRECTIONS, endpoint=False)]
    scripts += [[(cycles, (0.0, 0.0, params.max_wz_rad_s))], [(cycles, (0.0, 0.0, -params.max_wz_rad_s))],
                [(cycles, (params.max_vx_m_s, 0.0, params.max_wz_rad_s))]]
    reports = [check(simulate(geometry, params, s), geometry, joint_map, mass_kg, **limits) for s in scripts]
    violations = sorted({v for r in reports for v in r['violations']})
    warnings = sorted({w for r in reports for w in r['warnings']})
    return dict(ok=not violations, violations=violations, warnings=warnings, gait=params.gait,
                cases=len(reports),
                min_stability_margin_m=min(r.get('min_stability_margin_m', float('-inf')) for r in reports),
                peak_torque_nm=max(r.get('peak_torque_nm', 0.0) for r in reports),
                peak_motor_speed_rad_s=max(r.get('peak_motor_speed_rad_s', 0.0) for r in reports))
