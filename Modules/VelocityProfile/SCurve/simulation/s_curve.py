# -*- coding: utf-8 -*-

from __future__ import annotations

import math
from dataclasses import dataclass, field

S_CURVE_MAX_BS_ERROR = 0.001
EPS = 1.0e-6
RECOVERY_SEARCH_STEPS = 96
RECOVERY_REFINE_STEPS = 24


@dataclass(slots=True)
class BoundaryState:
    x: float
    v: float
    a: float


@dataclass(slots=True)
class SidePrepare:
    t_pre: float = 0.0
    x_pre: float = 0.0
    v_base: float = 0.0
    t_shift: float = 0.0
    vp_min: float = 0.0
    valid: bool = True


@dataclass(slots=True)
class FastEvalConfig:
    am: float
    jm: float
    am_square: float
    jerk_ramp_time: float
    inv_double_am: float


@dataclass(slots=True)
class FastEvalSide:
    v_base: float
    x_pre: float
    t_shift: float


@dataclass(slots=True)
class FastEvalProfile:
    has_uniform: bool = False
    v_base: float = 0.0
    vp: float = 0.0
    t1: float = 0.0
    t2: float = 0.0
    x1: float = 0.0
    v1: float = 0.0
    total_time: float = 0.0
    total_distance: float = 0.0


@dataclass(slots=True)
class FastEvalResult:
    dx1: float = 0.0
    dx3: float = 0.0
    delta: float = 0.0


@dataclass(slots=True)
class PrefixPlan:
    x0: float = 0.0
    v0: float = 0.0
    a0: float = 0.0
    j_ramp: float = 0.0
    j_settle: float = 0.0
    t_ramp: float = 0.0
    t_hold: float = 0.0
    t_settle: float = 0.0
    x_ramp: float = 0.0
    v_ramp: float = 0.0
    a_ramp: float = 0.0
    x_hold: float = 0.0
    v_hold: float = 0.0
    a_hold: float = 0.0
    total_time: float = 0.0
    end_x: float = 0.0
    end_v: float = 0.0
    end_a: float = 0.0
    valid: bool = False


@dataclass(slots=True)
class MotionCore:
    valid: bool = False
    has_const: bool = False
    direction: float = 1.0
    xs: float = 0.0
    xe: float = 0.0
    vs: float = 0.0
    as_: float = 0.0
    ve: float = 0.0
    ae: float = 0.0
    jm: float = 0.0
    vp: float = 0.0
    t1_pre: float = 0.0
    x1_pre: float = 0.0
    ts1: float = 0.0
    xs1: float = 0.0
    t3_pre: float = 0.0
    x3_pre: float = 0.0
    ts3: float = 0.0
    xs3: float = 0.0
    t1: float = 0.0
    t2: float = 0.0
    x1: float = 0.0
    total_time: float = 0.0
    process1: 'SCurveAccel' = field(default_factory=lambda: SCurveAccel())
    process3: 'SCurveAccel' = field(default_factory=lambda: SCurveAccel())


@dataclass(slots=True)
class SuffixPlan:
    reverse_plan: PrefixPlan = field(default_factory=PrefixPlan)
    start_x: float = 0.0
    valid: bool = False


def _sign(value: float) -> float:
    if value > 0.0:
        return 1.0
    if value < 0.0:
        return -1.0
    return 0.0


def _is_finite(*values: float) -> bool:
    return all(math.isfinite(v) for v in values)


def _evaluate_constant_jerk(state: BoundaryState, jerk: float, dt: float) -> BoundaryState:
    dt2 = dt * dt
    dt3 = dt2 * dt
    return BoundaryState(
        x=state.x + state.v * dt + 0.5 * state.a * dt2 + (1.0 / 6.0) * jerk * dt3,
        v=state.v + state.a * dt + 0.5 * jerk * dt2,
        a=state.a + jerk * dt,
    )


def _build_fast_eval_profile(cfg: FastEvalConfig, v_base: float, vp: float) -> FastEvalProfile:
    profile = FastEvalProfile(v_base=v_base, vp=vp)
    delta_v = vp - v_base
    if delta_v <= 0.0:
        return profile

    if cfg.jm * delta_v > cfg.am_square:
        profile.has_uniform = True
        profile.t1 = cfg.jerk_ramp_time
        profile.t2 = delta_v / cfg.am
        profile.v1 = v_base + 0.5 * cfg.am * profile.t1
        profile.x1 = v_base * profile.t1 + (1.0 / 6.0) * cfg.am * profile.t1 * profile.t1
        profile.total_time = profile.t2 + profile.t1
        profile.total_distance = (
            (vp * vp - v_base * v_base) * cfg.inv_double_am +
            0.5 * (v_base + vp) * profile.t1
        )
        return profile

    peak_acc = math.sqrt(cfg.jm * delta_v)
    profile.t1 = peak_acc / cfg.jm
    profile.t2 = profile.t1
    profile.v1 = v_base + 0.5 * peak_acc * peak_acc / cfg.jm
    profile.x1 = v_base * profile.t1 + (1.0 / 6.0) * peak_acc * profile.t1 * profile.t1
    profile.total_time = 2.0 * profile.t1
    profile.total_distance = (v_base + vp) * profile.t1
    return profile


def _evaluate_fast_eval_distance(cfg: FastEvalConfig, profile: FastEvalProfile, t: float) -> float:
    if t <= 0.0:
        return 0.0
    if t < profile.t1:
        return profile.v_base * t + (1.0 / 6.0) * cfg.jm * t * t * t
    if profile.has_uniform and t < profile.t2:
        dt = t - profile.t1
        return profile.x1 + profile.v1 * dt + 0.5 * cfg.am * dt * dt
    if t < profile.total_time:
        dt = profile.total_time - t
        return profile.total_distance - profile.vp * dt + (1.0 / 6.0) * cfg.jm * dt * dt * dt
    return profile.total_distance


def _evaluate_side_distance(cfg: FastEvalConfig, side: FastEvalSide, vp: float) -> float:
    if vp <= side.v_base:
        return side.x_pre
    profile = _build_fast_eval_profile(cfg, side.v_base, vp)
    shift_distance = _evaluate_fast_eval_distance(cfg, profile, side.t_shift)
    return side.x_pre + profile.total_distance - shift_distance


def _evaluate_distance_delta(
        cfg: FastEvalConfig,
        start: FastEvalSide,
        end: FastEvalSide,
        length: float,
        vp: float) -> FastEvalResult:
    dx1 = _evaluate_side_distance(cfg, start, vp)
    dx3 = _evaluate_side_distance(cfg, end, vp)
    return FastEvalResult(dx1=dx1, dx3=dx3, delta=dx1 + dx3 - length)


class SCurveAccel:
    def __init__(self):
        self.has_uniform = False
        self.vs = 0.0
        self.jm = 0.0
        self.total_time = 0.0
        self.total_distance = 0.0
        self.t1 = 0.0
        self.x1 = 0.0
        self.v1 = 0.0
        self.t2 = 0.0
        self.ap = 0.0
        self.vp = 0.0

    def init(self, vs: float, vp: float, am: float, jm: float) -> None:
        self.has_uniform = False
        self.vs = vs
        self.jm = jm
        self.total_time = 0.0
        self.total_distance = 0.0
        self.t1 = 0.0
        self.x1 = 0.0
        self.v1 = vs
        self.t2 = 0.0
        self.ap = 0.0
        self.vp = vp

        delta_v = vp - vs
        if delta_v <= 0.0:
            return

        self.has_uniform = jm * delta_v > am * am
        if self.has_uniform:
            self.ap = am
            self.t1 = am / jm
            self.t2 = delta_v / am
            self.v1 = vs + 0.5 * am * self.t1
            self.x1 = vs * self.t1 + (1.0 / 6.0) * am * self.t1 * self.t1
            self.total_time = self.t2 + self.t1
            self.total_distance = ((vp * vp - vs * vs) / (2.0 * am) +
                                   0.5 * (vs + vp) * self.t1)
            return

        self.ap = math.sqrt(jm * delta_v)
        self.t1 = self.ap / jm
        self.t2 = self.t1
        self.v1 = vs + 0.5 * self.ap * self.ap / jm
        self.x1 = vs * self.t1 + (1.0 / 6.0) * self.ap * self.t1 * self.t1
        self.total_time = 2.0 * self.t1
        self.total_distance = (vs + vp) * self.t1

    def get_distance(self, t: float) -> float:
        if t <= 0.0:
            return 0.0
        if t < self.t1:
            return self.vs * t + (1.0 / 6.0) * self.jm * t * t * t
        if self.has_uniform and t < self.t2:
            dt = t - self.t1
            return self.x1 + self.v1 * dt + 0.5 * self.ap * dt * dt
        if t < self.total_time:
            dt = self.total_time - t
            return self.total_distance - self.vp * dt + (1.0 / 6.0) * self.jm * dt * dt * dt
        return self.total_distance

    def get_velocity(self, t: float) -> float:
        if t <= 0.0:
            return self.vs
        if t < self.t1:
            return self.vs + 0.5 * self.jm * t * t
        if self.has_uniform and t < self.t2:
            return self.v1 + self.ap * (t - self.t1)
        if t < self.total_time:
            dt = self.total_time - t
            return self.vp - 0.5 * self.jm * dt * dt
        return self.vp

    def get_acceleration(self, t: float) -> float:
        if t <= 0.0:
            return 0.0
        if t < self.t1:
            return self.jm * t
        if self.has_uniform and t < self.t2:
            return self.ap
        if t < self.total_time:
            return self.jm * (self.total_time - t)
        return 0.0


class SCurve:
    S_CURVE_FAILED = 0
    S_CURVE_SUCCESS = 1

    def __init__(self):
        self._reset_state()

    def _reset_state(self) -> None:
        self.success = False
        self.total_time = 0.0
        self.prefix = PrefixPlan()
        self.main_core = MotionCore()
        self.suffix = SuffixPlan()
        self.debug = {
            "used_prefix": False,
            "used_suffix": False,
            "used_recovery_prefix": False,
            "used_clamp_prefix": False,
            "fallback_stop_prefix": False,
        }

    @staticmethod
    def _prepare_side(v0: float, a0: float, vm: float, jm: float) -> SidePrepare:
        result = SidePrepare()
        if a0 < 0.0:
            result.v_base = v0 - 0.5 * a0 * a0 / jm
            if abs(result.v_base) > vm:
                result.valid = False
                return result
            result.vp_min = result.v_base
            result.t_pre = -a0 / jm
            result.x_pre = v0 * result.t_pre + (1.0 / 3.0) * a0 * result.t_pre * result.t_pre
            return result

        result.vp_min = v0 + 0.5 * a0 * a0 / jm
        if vm < result.vp_min:
            result.valid = False
            return result
        result.t_shift = a0 / jm
        result.v_base = v0 - 0.5 * a0 * result.t_shift
        if result.vp_min < 0.0:
            result.vp_min = 0.0
        return result

    @staticmethod
    def _build_stop_prefix(start: BoundaryState, am: float, jm: float) -> PrefixPlan:
        plan = PrefixPlan()
        if jm <= 0.0 or am <= 0.0:
            return plan

        motion_sign = _sign(start.v)
        if motion_sign == 0.0:
            motion_sign = _sign(start.a)
        if motion_sign == 0.0 and abs(start.a) <= EPS:
            plan.x0 = start.x
            plan.v0 = start.v
            plan.a0 = start.a
            plan.x_ramp = start.x
            plan.v_ramp = start.v
            plan.a_ramp = start.a
            plan.x_hold = start.x
            plan.v_hold = start.v
            plan.a_hold = 0.0
            plan.end_x = start.x
            plan.end_v = 0.0
            plan.end_a = 0.0
            plan.valid = True
            return plan

        target_acc = -motion_sign * am
        j1 = jm if target_acc >= start.a else -jm
        t_ramp = abs(target_acc - start.a) / jm
        after_ramp = _evaluate_constant_jerk(start, j1, t_ramp)
        a_hold = after_ramp.a

        t_hold = 0.0
        if abs(a_hold) > EPS:
            v_settle = 0.5 * a_hold * a_hold / jm
            if motion_sign > 0.0 and after_ramp.v > v_settle:
                t_hold = abs((v_settle - after_ramp.v) / a_hold)
            elif motion_sign < 0.0 and after_ramp.v < -v_settle:
                t_hold = abs((-v_settle - after_ramp.v) / a_hold)

        after_hold = _evaluate_constant_jerk(after_ramp, 0.0, t_hold)
        j3 = -j1
        t_settle = abs(after_hold.a) / jm
        end = _evaluate_constant_jerk(after_hold, j3, t_settle)

        plan.x0 = start.x
        plan.v0 = start.v
        plan.a0 = start.a
        plan.j_ramp = j1
        plan.j_settle = j3
        plan.t_ramp = t_ramp
        plan.t_hold = t_hold
        plan.t_settle = t_settle
        plan.x_ramp = after_ramp.x
        plan.v_ramp = after_ramp.v
        plan.a_ramp = after_ramp.a
        plan.x_hold = after_hold.x
        plan.v_hold = after_hold.v
        plan.a_hold = after_hold.a
        plan.total_time = t_ramp + t_hold + t_settle
        plan.end_x = end.x
        plan.end_v = end.v
        plan.end_a = end.a
        plan.valid = _is_finite(plan.total_time, plan.end_x, plan.end_v, plan.end_a)
        return plan

    @staticmethod
    def _sample_prefix_x(plan: PrefixPlan, t: float) -> float:
        if not plan.valid or t <= 0.0:
            return plan.x0
        if t < plan.t_ramp:
            return (plan.x0 + plan.v0 * t + 0.5 * plan.a0 * t * t +
                    (1.0 / 6.0) * plan.j_ramp * t * t * t)
        if t < plan.t_ramp + plan.t_hold:
            dt = t - plan.t_ramp
            return plan.x_ramp + plan.v_ramp * dt + 0.5 * plan.a_ramp * dt * dt
        if t < plan.total_time:
            dt = t - plan.t_ramp - plan.t_hold
            return (plan.x_hold + plan.v_hold * dt + 0.5 * plan.a_hold * dt * dt +
                    (1.0 / 6.0) * plan.j_settle * dt * dt * dt)
        return plan.end_x

    @staticmethod
    def _sample_prefix_v(plan: PrefixPlan, t: float) -> float:
        if not plan.valid or t <= 0.0:
            return plan.v0
        if t < plan.t_ramp:
            return plan.v0 + plan.a0 * t + 0.5 * plan.j_ramp * t * t
        if t < plan.t_ramp + plan.t_hold:
            dt = t - plan.t_ramp
            return plan.v_ramp + plan.a_ramp * dt
        if t < plan.total_time:
            dt = t - plan.t_ramp - plan.t_hold
            return plan.v_hold + plan.a_hold * dt + 0.5 * plan.j_settle * dt * dt
        return plan.end_v

    @staticmethod
    def _sample_prefix_a(plan: PrefixPlan, t: float) -> float:
        if not plan.valid or t <= 0.0:
            return plan.a0
        if t < plan.t_ramp:
            return plan.a0 + plan.j_ramp * t
        if t < plan.t_ramp + plan.t_hold:
            return plan.a_ramp
        if t < plan.total_time:
            dt = t - plan.t_ramp - plan.t_hold
            return plan.a_hold + plan.j_settle * dt
        return plan.end_a

    @staticmethod
    def _shift_prefix_with_velocity_bias(
            plan: PrefixPlan,
            x_origin: float,
            velocity_bias: float) -> PrefixPlan:
        shifted = PrefixPlan()
        if not plan.valid:
            return shifted

        t_hold_end = plan.t_ramp + plan.t_hold
        shifted.x0 = x_origin
        shifted.v0 = velocity_bias + plan.v0
        shifted.a0 = plan.a0
        shifted.j_ramp = plan.j_ramp
        shifted.j_settle = plan.j_settle
        shifted.t_ramp = plan.t_ramp
        shifted.t_hold = plan.t_hold
        shifted.t_settle = plan.t_settle
        shifted.x_ramp = x_origin + (plan.x_ramp - plan.x0) + velocity_bias * plan.t_ramp
        shifted.v_ramp = velocity_bias + plan.v_ramp
        shifted.a_ramp = plan.a_ramp
        shifted.x_hold = x_origin + (plan.x_hold - plan.x0) + velocity_bias * t_hold_end
        shifted.v_hold = velocity_bias + plan.v_hold
        shifted.a_hold = plan.a_hold
        shifted.total_time = plan.total_time
        shifted.end_x = x_origin + (plan.end_x - plan.x0) + velocity_bias * plan.total_time
        shifted.end_v = velocity_bias + plan.end_v
        shifted.end_a = plan.end_a
        shifted.valid = _is_finite(
            shifted.total_time,
            shifted.end_x,
            shifted.end_v,
            shifted.end_a,
        )
        return shifted

    @classmethod
    def _build_velocity_clamp_prefix(
            cls,
            start: BoundaryState,
            target_v: float,
            am: float,
            jm: float) -> PrefixPlan:
        motion_sign = _sign(target_v)
        if motion_sign > 0.0 and start.v <= target_v + EPS and start.a > EPS:
            t_zero = start.a / jm
            peak_state = _evaluate_constant_jerk(start, -jm, t_zero)
            tail = cls._build_velocity_clamp_prefix(peak_state, target_v, am, jm)
            if not tail.valid or tail.j_ramp >= 0.0:
                return PrefixPlan()
            return cls._merge_prefix_with_leading_jerk(start, -jm, t_zero, tail)

        if motion_sign < 0.0 and start.v >= target_v - EPS and start.a < -EPS:
            t_zero = -start.a / jm
            peak_state = _evaluate_constant_jerk(start, jm, t_zero)
            tail = cls._build_velocity_clamp_prefix(peak_state, target_v, am, jm)
            if not tail.valid or tail.j_ramp <= 0.0:
                return PrefixPlan()
            return cls._merge_prefix_with_leading_jerk(start, jm, t_zero, tail)

        offset_plan = cls._build_stop_prefix(
            BoundaryState(0.0, start.v - target_v, start.a),
            am,
            jm,
        )
        if not offset_plan.valid:
            return PrefixPlan()
        return cls._shift_prefix_with_velocity_bias(offset_plan, start.x, target_v)

    @staticmethod
    def _merge_prefix_with_leading_jerk(
            start: BoundaryState,
            jerk: float,
            lead_time: float,
            tail: PrefixPlan) -> PrefixPlan:
        merged = PrefixPlan()
        merged.x0 = start.x
        merged.v0 = start.v
        merged.a0 = start.a
        merged.j_ramp = jerk
        merged.j_settle = tail.j_settle
        merged.t_ramp = lead_time + tail.t_ramp
        merged.t_hold = tail.t_hold
        merged.t_settle = tail.t_settle

        after_ramp = _evaluate_constant_jerk(start, jerk, merged.t_ramp)
        after_hold = _evaluate_constant_jerk(after_ramp, 0.0, merged.t_hold)
        end = _evaluate_constant_jerk(after_hold, merged.j_settle, merged.t_settle)

        merged.x_ramp = after_ramp.x
        merged.v_ramp = after_ramp.v
        merged.a_ramp = after_ramp.a
        merged.x_hold = after_hold.x
        merged.v_hold = after_hold.v
        merged.a_hold = after_hold.a
        merged.total_time = merged.t_ramp + merged.t_hold + merged.t_settle
        merged.end_x = end.x
        merged.end_v = end.v
        merged.end_a = end.a
        merged.valid = _is_finite(
            merged.total_time,
            merged.end_x,
            merged.end_v,
            merged.end_a,
        )
        return merged

    @staticmethod
    def _get_velocity_clamp_target(start: BoundaryState, vm: float, jm: float) -> float | None:
        motion_sign = _sign(start.v)
        if motion_sign == 0.0:
            motion_sign = _sign(start.a)
        if motion_sign == 0.0:
            return None

        if motion_sign > 0.0:
            if start.v > vm + EPS:
                return vm
            if start.a > EPS and start.v + 0.5 * start.a * start.a / jm > vm + EPS:
                return vm
            return None

        if start.v < -vm - EPS:
            return -vm
        if start.a < -EPS and start.v - 0.5 * start.a * start.a / jm < -vm - EPS:
            return -vm
        return None

    @staticmethod
    def _precheck_core(
            start: BoundaryState,
            end: BoundaryState,
            vm: float,
            jm: float) -> bool:
        direction = 1.0 if end.x >= start.x else -1.0
        side_start = SCurve._prepare_side(start.v * direction, start.a * direction, vm, jm)
        side_end = SCurve._prepare_side(end.v * direction, -end.a * direction, vm, jm)
        if not side_start.valid or not side_end.valid:
            return False

        vp_min = max(side_start.vp_min, side_end.vp_min, 0.0)
        return vp_min <= vm

    @classmethod
    def _trim_prefix(cls, plan: PrefixPlan, cut_time: float) -> PrefixPlan:
        trimmed = PrefixPlan()
        if not plan.valid:
            return trimmed

        t_end = min(max(cut_time, 0.0), plan.total_time)
        end_x = cls._sample_prefix_x(plan, t_end)
        end_v = cls._sample_prefix_v(plan, t_end)
        end_a = cls._sample_prefix_a(plan, t_end)

        trimmed.x0 = plan.x0
        trimmed.v0 = plan.v0
        trimmed.a0 = plan.a0
        trimmed.j_ramp = plan.j_ramp
        trimmed.j_settle = plan.j_settle
        trimmed.x_ramp = plan.x_ramp
        trimmed.v_ramp = plan.v_ramp
        trimmed.a_ramp = plan.a_ramp
        trimmed.x_hold = plan.x_hold
        trimmed.v_hold = plan.v_hold
        trimmed.a_hold = plan.a_hold
        trimmed.total_time = t_end
        trimmed.end_x = end_x
        trimmed.end_v = end_v
        trimmed.end_a = end_a
        trimmed.valid = _is_finite(t_end, end_x, end_v, end_a)

        if t_end <= EPS:
            trimmed.j_ramp = 0.0
            trimmed.j_settle = 0.0
            trimmed.t_ramp = 0.0
            trimmed.t_hold = 0.0
            trimmed.t_settle = 0.0
            trimmed.x_ramp = plan.x0
            trimmed.v_ramp = plan.v0
            trimmed.a_ramp = plan.a0
            trimmed.x_hold = plan.x0
            trimmed.v_hold = plan.v0
            trimmed.a_hold = plan.a0
            return trimmed

        ramp_end = plan.t_ramp
        hold_end = plan.t_ramp + plan.t_hold
        if t_end < ramp_end - EPS:
            trimmed.t_ramp = t_end
            trimmed.t_hold = 0.0
            trimmed.t_settle = 0.0
            trimmed.j_settle = 0.0
            trimmed.x_ramp = end_x
            trimmed.v_ramp = end_v
            trimmed.a_ramp = end_a
            trimmed.x_hold = end_x
            trimmed.v_hold = end_v
            trimmed.a_hold = end_a
            return trimmed

        if t_end < hold_end - EPS:
            trimmed.t_ramp = plan.t_ramp
            trimmed.t_hold = t_end - plan.t_ramp
            trimmed.t_settle = 0.0
            trimmed.j_settle = 0.0
            trimmed.x_hold = end_x
            trimmed.v_hold = end_v
            trimmed.a_hold = end_a
            return trimmed

        trimmed.t_ramp = plan.t_ramp
        trimmed.t_hold = plan.t_hold
        trimmed.t_settle = t_end - hold_end
        return trimmed

    def _try_velocity_clamp_recovery(
            self,
            start: BoundaryState,
            core_end: BoundaryState,
            vm: float,
            am: float,
            jm: float) -> tuple[PrefixPlan | None, MotionCore | None]:
        target_v = self._get_velocity_clamp_target(start, vm, jm)
        if target_v is None:
            return None, None

        prefix = self._build_velocity_clamp_prefix(start, target_v, am, jm)
        if not prefix.valid:
            return None, None

        recovered = BoundaryState(prefix.end_x, prefix.end_v, prefix.end_a)
        core = self._solve_core(recovered, core_end, vm, am, jm)
        if core is None:
            return None, None
        return prefix, core

    def _try_prefix_handoff(
            self,
            prefix_seed: PrefixPlan,
            core_end: BoundaryState,
            vm: float,
            am: float,
            jm: float) -> tuple[PrefixPlan | None, MotionCore | None]:
        if not prefix_seed.valid:
            return None, None

        prev_t = 0.0
        for step in range(1, RECOVERY_SEARCH_STEPS + 1):
            t = prefix_seed.total_time * step / RECOVERY_SEARCH_STEPS
            state = BoundaryState(
                x=self._sample_prefix_x(prefix_seed, t),
                v=self._sample_prefix_v(prefix_seed, t),
                a=self._sample_prefix_a(prefix_seed, t),
            )
            if not self._precheck_core(state, core_end, vm, jm):
                prev_t = t
                continue
            core = self._solve_core(state, core_end, vm, am, jm)
            if core is None:
                prev_t = t
                continue

            lo = prev_t
            hi = t
            hi_core = core
            for _ in range(RECOVERY_REFINE_STEPS):
                mid = 0.5 * (lo + hi)
                mid_state = BoundaryState(
                    x=self._sample_prefix_x(prefix_seed, mid),
                    v=self._sample_prefix_v(prefix_seed, mid),
                    a=self._sample_prefix_a(prefix_seed, mid),
                )
                if not self._precheck_core(mid_state, core_end, vm, jm):
                    lo = mid
                    continue
                mid_core = self._solve_core(mid_state, core_end, vm, am, jm)
                if mid_core is None:
                    lo = mid
                    continue
                hi = mid
                hi_core = mid_core

            return self._trim_prefix(prefix_seed, hi), hi_core

        return None, None

    def _solve_core(
            self,
            start: BoundaryState,
            end: BoundaryState,
            vm: float,
            am: float,
            jm: float) -> MotionCore | None:
        core = MotionCore()
        direction = 1.0 if end.x >= start.x else -1.0
        length = abs(end.x - start.x)

        core.direction = direction
        core.xs = start.x
        core.xe = end.x
        core.vs = start.v * direction
        core.as_ = start.a * direction
        core.ve = end.v * direction
        core.ae = end.a * direction
        core.jm = jm

        side_start = self._prepare_side(core.vs, core.as_, vm, jm)
        side_end = self._prepare_side(core.ve, -core.ae, vm, jm)
        if not side_start.valid or not side_end.valid:
            return None

        vp_min = max(side_start.vp_min, side_end.vp_min, 0.0)
        if vm < vp_min:
            return None

        core.t1_pre = side_start.t_pre
        core.x1_pre = core.xs + direction * side_start.x_pre
        core.ts1 = side_start.t_shift
        core.t3_pre = side_end.t_pre
        core.x3_pre = side_end.x_pre
        core.ts3 = side_end.t_shift

        len0 = length - side_start.x_pre - side_end.x_pre
        if len0 < -S_CURVE_MAX_BS_ERROR:
            return None

        fast_eval_cfg = FastEvalConfig(
            am=am,
            jm=jm,
            am_square=am * am,
            jerk_ramp_time=am / jm,
            inv_double_am=0.5 / am,
        )
        start_eval = FastEvalSide(side_start.v_base, side_start.x_pre, side_start.t_shift)
        end_eval = FastEvalSide(side_end.v_base, side_end.x_pre, side_end.t_shift)

        min_eval = _evaluate_distance_delta(fast_eval_cfg, start_eval, end_eval, length, vp_min)
        if min_eval.delta > S_CURVE_MAX_BS_ERROR:
            return None

        vm_eval = _evaluate_distance_delta(fast_eval_cfg, start_eval, end_eval, length, vm)
        x_const = -vm_eval.delta
        if x_const > 0.0:
            core.process1.init(side_start.v_base, vm, am, jm)
            core.process3.init(side_end.v_base, vm, am, jm)
            core.xs1 = core.process1.get_distance(core.ts1)
            core.xs3 = core.process3.get_distance(core.ts3)
            core.has_const = True
            core.t1 = core.t1_pre + core.process1.total_time - core.ts1
            core.t2 = core.t1 + x_const / vm
            core.total_time = core.t2 + core.t3_pre + core.process3.total_time - core.ts3
            core.x1 = core.xs + core.direction * vm_eval.dx1
            core.vp = vm
            core.valid = True
            return core

        l = vp_min
        r = vm
        l_eval = min_eval
        while r - l > S_CURVE_MAX_BS_ERROR:
            mid = 0.5 * (l + r)
            mid_eval = _evaluate_distance_delta(fast_eval_cfg, start_eval, end_eval, length, mid)
            delta_d = mid_eval.delta
            if delta_d > 0.0:
                r = mid
            else:
                l = mid
                l_eval = mid_eval
                if abs(delta_d) <= S_CURVE_MAX_BS_ERROR:
                    break

        if l_eval.delta > S_CURVE_MAX_BS_ERROR:
            return None

        vp = l
        core.process1.init(side_start.v_base, vp, am, jm)
        core.process3.init(side_end.v_base, vp, am, jm)
        core.xs1 = core.process1.get_distance(core.ts1)
        core.xs3 = core.process3.get_distance(core.ts3)
        dx1 = side_start.x_pre + core.process1.total_distance - core.xs1
        dx3 = side_end.x_pre + core.process3.total_distance - core.xs3
        delta_d = dx1 + dx3 - length
        if delta_d > S_CURVE_MAX_BS_ERROR:
            return None

        residual_const = -delta_d if delta_d < 0.0 else 0.0
        core.has_const = residual_const > 0.0
        core.t1 = core.t1_pre + core.process1.total_time - core.ts1
        core.t2 = core.t1 + (residual_const / vp if core.has_const else 0.0)
        core.total_time = core.t2 + core.t3_pre + core.process3.total_time - core.ts3
        core.x1 = core.xs + core.direction * dx1
        core.vp = vp
        core.valid = True
        return core

    @staticmethod
    def _reverse_distance(core: MotionCore, tau: float) -> float:
        if tau <= 0.0:
            return 0.0
        if tau < core.t3_pre:
            return core.ve * tau - 0.5 * core.ae * tau * tau + (1.0 / 6.0) * core.jm * tau * tau * tau
        return core.x3_pre + core.process3.get_distance(tau - core.t3_pre + core.ts3) - core.xs3

    @staticmethod
    def _reverse_velocity(core: MotionCore, tau: float) -> float:
        if tau <= 0.0:
            return core.ve
        if tau < core.t3_pre:
            return core.ve - core.ae * tau + 0.5 * core.jm * tau * tau
        return core.process3.get_velocity(tau - core.t3_pre + core.ts3)

    @staticmethod
    def _reverse_acceleration(core: MotionCore, tau: float) -> float:
        if tau <= 0.0:
            return -core.ae
        if tau < core.t3_pre:
            return -core.ae + core.jm * tau
        return core.process3.get_acceleration(tau - core.t3_pre + core.ts3)

    def _sample_core_x(self, core: MotionCore, t: float) -> float:
        if t <= 0.0:
            return core.xs
        if t < core.t1_pre:
            return (core.xs + core.direction *
                    (core.vs * t + 0.5 * core.as_ * t * t + (1.0 / 6.0) * core.jm * t * t * t))
        if t < core.t1:
            return (core.x1_pre + core.direction *
                    (core.process1.get_distance(t - core.t1_pre + core.ts1) - core.xs1))
        if core.has_const and t < core.t2:
            return core.x1 + core.direction * core.vp * (t - core.t1)
        if t < core.total_time:
            return core.xe - core.direction * self._reverse_distance(core, core.total_time - t)
        return core.xe

    def _sample_core_v(self, core: MotionCore, t: float) -> float:
        if t <= 0.0:
            return core.direction * core.vs
        if t < core.t1_pre:
            return core.direction * (core.vs + core.as_ * t + 0.5 * core.jm * t * t)
        if t < core.t1:
            return core.direction * core.process1.get_velocity(t - core.t1_pre + core.ts1)
        if core.has_const and t < core.t2:
            return core.direction * core.vp
        if t < core.total_time:
            return core.direction * self._reverse_velocity(core, core.total_time - t)
        return core.direction * core.ve

    def _sample_core_a(self, core: MotionCore, t: float) -> float:
        if t <= 0.0:
            return core.direction * core.as_
        if t < core.t1_pre:
            return core.direction * (core.as_ + core.jm * t)
        if t < core.t1:
            return core.direction * core.process1.get_acceleration(t - core.t1_pre + core.ts1)
        if core.has_const and t < core.t2:
            return 0.0
        if t < core.total_time:
            return -core.direction * self._reverse_acceleration(core, core.total_time - t)
        return core.direction * core.ae

    def _sample_suffix_x(self, elapsed: float) -> float:
        tau = self.suffix.reverse_plan.total_time - elapsed
        return self.suffix.start_x + self.suffix.reverse_plan.end_x - self._sample_prefix_x(
            self.suffix.reverse_plan, tau)

    def _sample_suffix_v(self, elapsed: float) -> float:
        tau = self.suffix.reverse_plan.total_time - elapsed
        return self._sample_prefix_v(self.suffix.reverse_plan, tau)

    def _sample_suffix_a(self, elapsed: float) -> float:
        tau = self.suffix.reverse_plan.total_time - elapsed
        return -self._sample_prefix_a(self.suffix.reverse_plan, tau)

    def init(
            self,
            xs: float,
            xe: float,
            vs: float,
            as_: float,
            vm: float,
            am: float,
            jm: float,
            ve: float = 0.0,
            ae: float = 0.0) -> int:
        self._reset_state()

        if not _is_finite(xs, xe, vs, as_, vm, am, jm, ve, ae):
            return self.S_CURVE_FAILED

        vm = abs(vm)
        am = abs(am)
        jm = abs(jm)
        if vm <= 0.0 or am <= 0.0 or jm <= 0.0:
            return self.S_CURVE_FAILED
        if abs(ve) > vm or abs(ae) > am:
            return self.S_CURVE_FAILED

        start = BoundaryState(xs, vs, as_)
        end = BoundaryState(xe, ve, ae)

        if abs(vs) <= EPS and abs(as_) <= EPS and abs(xe - xs) <= EPS and abs(ve) <= EPS and abs(ae) <= EPS:
            self.main_core = MotionCore(valid=True, xs=xs, xe=xe, ve=ve, ae=ae)
            self.success = True
            return self.S_CURVE_SUCCESS

        reverse_end = self._build_stop_prefix(BoundaryState(0.0, ve, -ae), am, jm)
        core_end = BoundaryState(x=xe, v=ve, a=ae)
        if reverse_end.valid and (abs(ve) > EPS or abs(ae) > EPS):
            core_end = BoundaryState(
                x=xe - reverse_end.end_x,
                v=reverse_end.end_v,
                a=-reverse_end.end_a,
            )
            self.suffix = SuffixPlan(reverse_plan=reverse_end, start_x=core_end.x, valid=True)
            self.debug["used_suffix"] = True

        direct_core = self._solve_core(start, core_end, vm, am, jm)
        if direct_core is not None:
            self.main_core = direct_core
            self.success = True
            self.total_time = self.main_core.total_time + (
                self.suffix.reverse_plan.total_time if self.suffix.valid else 0.0
            )
            return self.S_CURVE_SUCCESS

        prefix, core = self._try_velocity_clamp_recovery(start, core_end, vm, am, jm)
        if prefix is not None and core is not None:
            self.prefix = prefix
            self.main_core = core
            self.debug["used_prefix"] = True
            self.debug["used_recovery_prefix"] = True
            self.debug["used_clamp_prefix"] = True
            self.success = True
            self.total_time = self.prefix.total_time + self.main_core.total_time + (
                self.suffix.reverse_plan.total_time if self.suffix.valid else 0.0
            )
            return self.S_CURVE_SUCCESS

        stop_prefix = self._build_stop_prefix(start, am, jm)
        if not stop_prefix.valid:
            return self.S_CURVE_FAILED

        prefix, core = self._try_prefix_handoff(stop_prefix, core_end, vm, am, jm)
        if prefix is None or core is None:
            return self.S_CURVE_FAILED

        self.prefix = prefix
        self.main_core = core
        self.debug["used_prefix"] = True
        if self.prefix.total_time < stop_prefix.total_time - S_CURVE_MAX_BS_ERROR:
            self.debug["used_recovery_prefix"] = True
        else:
            self.debug["fallback_stop_prefix"] = True
        self.success = True
        self.total_time = self.prefix.total_time + self.main_core.total_time + (
            self.suffix.reverse_plan.total_time if self.suffix.valid else 0.0
        )
        return self.S_CURVE_SUCCESS

    def calc_x(self, t: float) -> float:
        if not self.success:
            return 0.0
        if t <= 0.0:
            return self.prefix.x0 if self.prefix.valid else self.main_core.xs
        if self.prefix.valid and t < self.prefix.total_time:
            return self._sample_prefix_x(self.prefix, t)
        core_start = self.prefix.total_time if self.prefix.valid else 0.0
        core_end = core_start + self.main_core.total_time
        if t < core_end:
            return self._sample_core_x(self.main_core, t - core_start)
        if self.suffix.valid:
            return self._sample_suffix_x(t - core_end)
        return self.main_core.xe

    def calc_v(self, t: float) -> float:
        if not self.success:
            return 0.0
        if t <= 0.0:
            return self.prefix.v0 if self.prefix.valid else self.main_core.direction * self.main_core.vs
        if self.prefix.valid and t < self.prefix.total_time:
            return self._sample_prefix_v(self.prefix, t)
        core_start = self.prefix.total_time if self.prefix.valid else 0.0
        core_end = core_start + self.main_core.total_time
        if t < core_end:
            return self._sample_core_v(self.main_core, t - core_start)
        if self.suffix.valid:
            return self._sample_suffix_v(t - core_end)
        return self.main_core.direction * self.main_core.ve

    def calc_a(self, t: float) -> float:
        if not self.success:
            return 0.0
        if t <= 0.0:
            return self.prefix.a0 if self.prefix.valid else self.main_core.direction * self.main_core.as_
        if self.prefix.valid and t < self.prefix.total_time:
            return self._sample_prefix_a(self.prefix, t)
        core_start = self.prefix.total_time if self.prefix.valid else 0.0
        core_end = core_start + self.main_core.total_time
        if t < core_end:
            return self._sample_core_a(self.main_core, t - core_start)
        if self.suffix.valid:
            return self._sample_suffix_a(t - core_end)
        return self.main_core.direction * self.main_core.ae
