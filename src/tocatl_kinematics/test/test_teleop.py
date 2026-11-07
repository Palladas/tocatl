import pytest

from tocatl_kinematics.teleop import ARM_HOLD_S, JOY_TIMEOUT_S, PadLayout, TeleopMapper, shape

L = PadLayout()


def pad(lx=0.0, ly=0.0, rx=0.0, rt=1.0, **held):
    axes = [0.0] * 9
    axes[L.axis_lx], axes[L.axis_ly], axes[L.axis_rx], axes[L.axis_rt] = lx, ly, rx, rt
    buttons = [0] * 11
    for name in held:
        buttons[getattr(L, 'button_' + name)] = 1
    return axes, buttons


def test_sticks_only_move_the_robot_while_the_deadman_is_held():
    m = TeleopMapper()
    m.update(*pad(ly=1.0, lx=0.5, rx=-1.0), now=0.0)
    assert m.command(0.0) == (0.0, 0.0, 0.0)
    m.update(*pad(ly=1.0, rx=-1.0, lb=True), now=0.05)
    vx, vy, wz = m.command(0.05)
    assert vx == pytest.approx(0.3 * 0.05) and vy == 0.0 and wz == pytest.approx(-0.3 * 0.2)
    m.update(*pad(ly=1.0, rt=-1.0, lb=True), now=0.1)           # trigger fully pressed: full speed
    assert m.command(0.1)[0] == pytest.approx(0.05)
    m.update(*pad(ly=1.0), now=0.15)                            # LB released
    assert m.command(0.15) == (0.0, 0.0, 0.0)


def test_lost_joystick_stops_the_walk():
    m = TeleopMapper()
    m.update(*pad(ly=1.0, rt=-1.0, lb=True), now=0.0)
    assert m.command(JOY_TIMEOUT_S - 0.01)[0] > 0
    assert m.command(JOY_TIMEOUT_S + 0.01) == (0.0, 0.0, 0.0)
    assert TeleopMapper().command(0.0) == (0.0, 0.0, 0.0)


def test_deadzone_and_expo():
    assert shape(0.1, 0.12, 0.5) == 0.0
    assert shape(1.0, 0.12, 0.5) == pytest.approx(1.0) and shape(-1.0, 0.12, 0.5) == pytest.approx(-1.0)
    assert 0 < shape(0.5, 0.12, 0.5) < (0.5 - 0.12) / 0.88      # gentler than linear near centre


def test_buttons_fire_once_per_press_and_arm_needs_a_hold():
    m = TeleopMapper()
    m.update(*pad(b=True), now=0.0)
    m.update(*pad(b=True), now=0.05)                            # still held: no repeat
    m.update(*pad(x=True), now=0.1)
    m.update(*pad(), now=0.15)
    m.update(*pad(x=True), now=0.2)
    m.update(*pad(back=True), now=0.25)
    assert m.take_events() == ['stop', ('gait', 'tetrapod'), ('gait', 'tripod'), 'disarm']
    m.update(*pad(start=True), now=1.0)
    m.update(*pad(start=True), now=1.0 + ARM_HOLD_S / 2)
    assert m.take_events() == []                                # a tap does not arm
    m.update(*pad(start=True), now=1.0 + ARM_HOLD_S)
    m.update(*pad(start=True), now=1.0 + 2 * ARM_HOLD_S)
    assert m.take_events() == ['arm']                           # once per hold
    m.update(*pad(xbox=True), now=5.0)
    assert m.take_events() == ['disarm']


def test_short_messages_from_other_drivers_do_not_crash():
    m = TeleopMapper()
    m.update([0.0, 1.0], [0, 0, 0, 0, 1], now=0.0)               # only 2 axes, LB at index 4
    assert m.command(0.0)[0] > 0


def test_dpad_steps_height_once_per_press():
    m = TeleopMapper()
    axes, buttons = pad()
    for value, now in ((1.0, 0.0), (1.0, 0.05), (0.0, 0.1), (-1.0, 0.15), (-1.0, 0.2), (1.0, 0.25)):
        axes = list(axes)
        axes[L.axis_dpad_y] = value
        m.update(axes, buttons, now)
    assert m.take_events() == [('height', 1), ('height', -1), ('height', 1)]


def test_cue_buttons_fire_only_while_the_deadman_is_held():
    m = TeleopMapper()
    m.update(*pad(a=True), now=0.0)                       # A without LB: nothing
    m.update(*pad(), now=0.05)
    m.update(*pad(lb=True), now=0.1)
    m.update(*pad(lb=True, a=True), now=0.15)             # LB + A: wave, once per press
    m.update(*pad(lb=True, a=True), now=0.2)
    m.update(*pad(lb=True, y=True), now=0.25)
    m.update(*pad(lb=True, rb=True), now=0.3)
    m.update(*pad(lb=True, ls=True), now=0.35)
    m.update(*pad(lb=True, rs=True), now=0.4)
    assert m.take_events() == [('cue', 'wave'), ('cue', 'bloom'), ('cue', 'circle'), ('cue', 'alternate'),
                               ('cue', 'wave_reverse')]


def test_deadman_is_true_only_while_lb_is_held_and_the_pad_is_fresh():
    m = TeleopMapper()
    assert not m.deadman(0.0)                             # no /joy yet
    m.update(*pad(lb=True), now=0.0)
    assert m.deadman(0.1)
    assert not m.deadman(JOY_TIMEOUT_S + 0.01)           # pad went quiet
    m.update(*pad(), now=1.0)
    assert not m.deadman(1.0)                             # LB released
