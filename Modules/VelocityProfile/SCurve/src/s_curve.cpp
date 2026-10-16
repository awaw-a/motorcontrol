/**
 * @file    s_curve.cpp
 * @author  syhanjin LIJunHong659
 * @date    2026-04-09
 */
#include "s_curve.hpp"

#include <cmath>

namespace velocity_profile
{
namespace
{
constexpr float kHalf     = 0.5f;
constexpr float kOneSixth = 1.0f / 6.0f;
constexpr float kZero     = 0.0f;
constexpr float kEpsilon  = 1.0e-6f;
constexpr float kDistanceTolerance = S_CURVE_MAX_BS_ERROR;
constexpr int   kRecoverySearchSteps   = 96;
constexpr int   kRecoveryRefineSteps   = 24;
constexpr int   kPeakVelocityBisectIterations = 64;

struct FastEvalConfig
{
    float am;
    float jm;
    float am_square;
    float jerk_ramp_time;
    float inv_double_am;
};

struct FastEvalSide
{
    float v_base;
    float x_pre;
    float t_shift;
};

struct FastEvalProfile
{
    bool  has_uniform;
    float v_base;
    float vp;
    float t1;
    float t2;
    float x1;
    float v1;
    float total_time;
    float total_distance;
};

struct FastEvalResult
{
    float dx1;
    float dx3;
    float delta;
};

[[nodiscard]] FastEvalProfile BuildFastEvalProfile(
        const FastEvalConfig& cfg, const float v_base, const float vp)
{
    FastEvalProfile profile{};
    const float     delta_v = vp - v_base;

    profile.has_uniform = false;
    profile.v_base      = v_base;
    profile.vp          = vp;
    profile.t1          = 0;
    profile.t2          = 0;
    profile.x1          = 0;
    profile.v1          = v_base;
    profile.total_time  = 0;
    profile.total_distance = 0;

    if (delta_v <= 0)
        return profile;

    if (cfg.jm * delta_v > cfg.am_square)
    {
        profile.has_uniform    = true;
        profile.t1             = cfg.jerk_ramp_time;
        profile.t2             = delta_v / cfg.am;
        profile.v1             = v_base + kHalf * cfg.am * profile.t1;
        profile.x1             = v_base * profile.t1 + kOneSixth * cfg.am * profile.t1 * profile.t1;
        profile.total_time     = profile.t2 + profile.t1;
        profile.total_distance = (vp * vp - v_base * v_base) * cfg.inv_double_am +
                                 kHalf * (v_base + vp) * profile.t1;
        return profile;
    }

    const float peak_acc = sqrtf(cfg.jm * delta_v);
    profile.t1           = peak_acc / cfg.jm;
    profile.t2           = profile.t1;
    profile.v1           = v_base + kHalf * peak_acc * peak_acc / cfg.jm;
    profile.x1           = v_base * profile.t1 + kOneSixth * peak_acc * profile.t1 * profile.t1;
    profile.total_time   = 2.0f * profile.t1;
    profile.total_distance = (v_base + vp) * profile.t1;
    return profile;
}

[[nodiscard]] float EvaluateFastEvalDistance(
        const FastEvalConfig& cfg, const FastEvalProfile& profile, const float t)
{
    if (t <= kZero)
        return 0;

    if (t < profile.t1)
        return profile.v_base * t + kOneSixth * cfg.jm * t * t * t;

    if (profile.has_uniform && t < profile.t2)
    {
        const float dt = t - profile.t1;
        return profile.x1 + profile.v1 * dt + kHalf * cfg.am * dt * dt;
    }

    if (t < profile.total_time)
    {
        const float dt = profile.total_time - t;
        return profile.total_distance - profile.vp * dt + kOneSixth * cfg.jm * dt * dt * dt;
    }

    return profile.total_distance;
}

[[nodiscard]] float EvaluateSideDistance(
        const FastEvalConfig& cfg, const FastEvalSide& side, const float vp)
{
    if (vp <= side.v_base)
        return side.x_pre;

    const FastEvalProfile profile        = BuildFastEvalProfile(cfg, side.v_base, vp);
    const float           shift_distance = EvaluateFastEvalDistance(cfg, profile, side.t_shift);
    return side.x_pre + profile.total_distance - shift_distance;
}

[[nodiscard]] FastEvalResult EvaluateDistanceDelta(
        const FastEvalConfig& cfg, const FastEvalSide& start, const FastEvalSide& end, const float len,
        const float vp)
{
    FastEvalResult result{};
    result.dx1   = EvaluateSideDistance(cfg, start, vp);
    result.dx3   = EvaluateSideDistance(cfg, end, vp);
    result.delta = result.dx1 + result.dx3 - len;
    return result;
}
} // namespace

SCurveProfile::SCurveAccel::SCurveAccel() :
    has_uniform_(false), vs_(0), jm_(0), total_time_(0), total_distance_(0), t1_(0), x1_(0), v1_(0),
    t2_(0), ap_(0), vp_(0)
{
}

void SCurveProfile::SCurveAccel::init(const float vs, const float vp, const float am, const float jm)
{
    has_uniform_    = false;
    vs_             = vs;
    jm_             = jm;
    total_time_     = 0;
    total_distance_ = 0;
    t1_             = 0;
    x1_             = 0;
    v1_             = vs;
    t2_             = 0;
    ap_             = 0;
    vp_             = vp;

    const float delta_v = vp - vs;
    if (delta_v <= 0)
        return;

    has_uniform_ = jm * delta_v > am * am;
    if (has_uniform_)
    {
        ap_ = am;
        t1_ = am / jm;
        t2_ = delta_v / am;

        v1_ = vs + kHalf * am * t1_;
        x1_ = vs * t1_ + kOneSixth * am * t1_ * t1_;

        total_time_     = t2_ + t1_;
        total_distance_ = (vp * vp - vs * vs) / (2.0f * am) + kHalf * (vs + vp) * t1_;
        return;
    }

    ap_ = sqrtf(jm * delta_v);
    t1_ = ap_ / jm;
    t2_ = t1_;
    v1_ = vs + kHalf * ap_ * ap_ / jm;
    x1_ = vs * t1_ + kOneSixth * ap_ * t1_ * t1_;

    total_time_     = 2.0f * t1_;
    total_distance_ = (vs + vp) * t1_;
}

float SCurveProfile::SCurveAccel::getDistance(const float t) const
{
    if (t <= 0)
        return 0;
    if (t < t1_)
        return vs_ * t + kOneSixth * jm_ * t * t * t;
    if (has_uniform_ && t < t2_)
    {
        const float dt = t - t1_;
        return x1_ + v1_ * dt + kHalf * ap_ * dt * dt;
    }
    if (t < total_time_)
    {
        const float dt = total_time_ - t;
        return total_distance_ - vp_ * dt + kOneSixth * jm_ * dt * dt * dt;
    }
    return total_distance_;
}

float SCurveProfile::SCurveAccel::getVelocity(const float t) const
{
    if (t <= 0)
        return vs_;
    if (t < t1_)
        return vs_ + kHalf * jm_ * t * t;
    if (has_uniform_ && t < t2_)
        return v1_ + ap_ * (t - t1_);
    if (t < total_time_)
    {
        const float dt = total_time_ - t;
        return vp_ - kHalf * jm_ * dt * dt;
    }
    return vp_;
}

float SCurveProfile::SCurveAccel::getAcceleration(const float t) const
{
    if (t <= 0)
        return 0;
    if (t < t1_)
        return jm_ * t;
    if (has_uniform_ && t < t2_)
        return ap_;
    if (t < total_time_)
        return jm_ * (total_time_ - t);
    return 0;
}

bool SCurveProfile::IsFinite(const float value)
{
    return std::isfinite(value) != 0;
}

float SCurveProfile::Sign(const float value)
{
    if (value > 0)
        return 1.0f;
    if (value < 0)
        return -1.0f;
    return 0.0f;
}

SCurveProfile::BoundaryState SCurveProfile::EvaluateConstantJerk(
        const BoundaryState& state, const float jerk, const float dt)
{
    const float dt2 = dt * dt;
    const float dt3 = dt2 * dt;
    return {
        state.x + state.v * dt + kHalf * state.a * dt2 + kOneSixth * jerk * dt3,
        state.v + state.a * dt + kHalf * jerk * dt2,
        state.a + jerk * dt,
    };
}

SCurveProfile::PrefixPlan SCurveProfile::BuildStopPrefix(
        const BoundaryState& start, const float am, const float jm)
{
    PrefixPlan plan{};
    plan.valid = false;

    if (jm <= 0 || am <= 0)
        return plan;

    float motion_sign = Sign(start.v);
    if (motion_sign == 0.0f)
        motion_sign = Sign(start.a);
    if (motion_sign == 0.0f && fabsf(start.a) <= kEpsilon)
    {
        plan.x0        = start.x;
        plan.v0        = start.v;
        plan.a0        = start.a;
        plan.j_ramp    = 0;
        plan.j_settle  = 0;
        plan.t_ramp    = 0;
        plan.t_hold    = 0;
        plan.t_settle  = 0;
        plan.x_ramp    = start.x;
        plan.v_ramp    = start.v;
        plan.a_ramp    = start.a;
        plan.x_hold    = start.x;
        plan.v_hold    = start.v;
        plan.a_hold    = 0;
        plan.total_time = 0;
        plan.end_x     = start.x;
        plan.end_v     = 0;
        plan.end_a     = 0;
        plan.valid     = true;
        return plan;
    }

    const float target_acc = -motion_sign * am;
    const float j1         = target_acc >= start.a ? jm : -jm;
    float       t_ramp     = fabsf(target_acc - start.a) / jm;

    BoundaryState after_ramp = EvaluateConstantJerk(start, j1, t_ramp);
    float         a_hold     = after_ramp.a;

    float t_hold = 0;
    if (fabsf(a_hold) > kEpsilon)
    {
        const float hold_sign = Sign(a_hold);
        if (hold_sign == motion_sign)
        {
            t_hold = 0;
        }
        else
        {
            const float v_settle = kHalf * a_hold * a_hold / jm;
            if ((motion_sign > 0 && after_ramp.v > v_settle) || (motion_sign < 0 && after_ramp.v < -v_settle))
            {
                t_hold = fabsf((motion_sign > 0 ? (v_settle - after_ramp.v) : (-v_settle - after_ramp.v)) / a_hold);
            }
        }
    }

    BoundaryState after_hold = EvaluateConstantJerk(after_ramp, 0.0f, t_hold);
    const float   j3         = -j1;
    const float   t_settle   = fabsf(after_hold.a) / jm;
    BoundaryState end        = EvaluateConstantJerk(after_hold, j3, t_settle);

    plan.x0        = start.x;
    plan.v0        = start.v;
    plan.a0        = start.a;
    plan.j_ramp    = j1;
    plan.j_settle  = j3;
    plan.t_ramp    = t_ramp;
    plan.t_hold    = t_hold;
    plan.t_settle  = t_settle;
    plan.x_ramp    = after_ramp.x;
    plan.v_ramp    = after_ramp.v;
    plan.a_ramp    = after_ramp.a;
    plan.x_hold    = after_hold.x;
    plan.v_hold    = after_hold.v;
    plan.a_hold    = after_hold.a;
    plan.total_time = t_ramp + t_hold + t_settle;
    plan.end_x     = end.x;
    plan.end_v     = end.v;
    plan.end_a     = end.a;
    plan.valid     = IsFinite(plan.total_time) && IsFinite(plan.end_x) && IsFinite(plan.end_v) && IsFinite(plan.end_a);
    return plan;
}

SCurveProfile::PrefixPlan SCurveProfile::ShiftPrefixWithVelocityBias(
        const PrefixPlan& plan, const float x_origin, const float velocity_bias)
{
    PrefixPlan shifted{};
    shifted.valid = false;
    if (!plan.valid)
        return shifted;

    const float t_hold_end = plan.t_ramp + plan.t_hold;
    shifted.x0        = x_origin;
    shifted.v0        = velocity_bias + plan.v0;
    shifted.a0        = plan.a0;
    shifted.j_ramp    = plan.j_ramp;
    shifted.j_settle  = plan.j_settle;
    shifted.t_ramp    = plan.t_ramp;
    shifted.t_hold    = plan.t_hold;
    shifted.t_settle  = plan.t_settle;
    shifted.x_ramp    = x_origin + (plan.x_ramp - plan.x0) + velocity_bias * plan.t_ramp;
    shifted.v_ramp    = velocity_bias + plan.v_ramp;
    shifted.a_ramp    = plan.a_ramp;
    shifted.x_hold    = x_origin + (plan.x_hold - plan.x0) + velocity_bias * t_hold_end;
    shifted.v_hold    = velocity_bias + plan.v_hold;
    shifted.a_hold    = plan.a_hold;
    shifted.total_time = plan.total_time;
    shifted.end_x     = x_origin + (plan.end_x - plan.x0) + velocity_bias * plan.total_time;
    shifted.end_v     = velocity_bias + plan.end_v;
    shifted.end_a     = plan.end_a;
    shifted.valid     = IsFinite(shifted.total_time) && IsFinite(shifted.end_x) && IsFinite(shifted.end_v) &&
                    IsFinite(shifted.end_a);
    return shifted;
}

SCurveProfile::PrefixPlan SCurveProfile::MergePrefixWithLeadingJerk(
        const BoundaryState& start, const float jerk, const float lead_time, const PrefixPlan& tail)
{
    PrefixPlan merged{};
    merged.valid = false;
    if (!tail.valid)
        return merged;

    merged.x0        = start.x;
    merged.v0        = start.v;
    merged.a0        = start.a;
    merged.j_ramp    = jerk;
    merged.j_settle  = tail.j_settle;
    merged.t_ramp    = lead_time + tail.t_ramp;
    merged.t_hold    = tail.t_hold;
    merged.t_settle  = tail.t_settle;

    const BoundaryState after_ramp = EvaluateConstantJerk(start, jerk, merged.t_ramp);
    const BoundaryState after_hold = EvaluateConstantJerk(after_ramp, 0.0f, merged.t_hold);
    const BoundaryState end        = EvaluateConstantJerk(after_hold, merged.j_settle, merged.t_settle);

    merged.x_ramp    = after_ramp.x;
    merged.v_ramp    = after_ramp.v;
    merged.a_ramp    = after_ramp.a;
    merged.x_hold    = after_hold.x;
    merged.v_hold    = after_hold.v;
    merged.a_hold    = after_hold.a;
    merged.total_time = merged.t_ramp + merged.t_hold + merged.t_settle;
    merged.end_x     = end.x;
    merged.end_v     = end.v;
    merged.end_a     = end.a;
    merged.valid     = IsFinite(merged.total_time) && IsFinite(merged.end_x) && IsFinite(merged.end_v) &&
                   IsFinite(merged.end_a);
    return merged;
}

SCurveProfile::PrefixPlan SCurveProfile::BuildVelocityClampPrefix(
        const BoundaryState& start, const float target_v, const float am, const float jm)
{
    const float motion_sign = Sign(target_v);
    if (motion_sign > 0.0f && start.v <= target_v + kEpsilon && start.a > kEpsilon)
    {
        const float         t_zero     = start.a / jm;
        const BoundaryState peak_state = EvaluateConstantJerk(start, -jm, t_zero);
        const PrefixPlan    tail       = BuildVelocityClampPrefix(peak_state, target_v, am, jm);
        if (!tail.valid || tail.j_ramp >= 0.0f)
            return {};
        return MergePrefixWithLeadingJerk(start, -jm, t_zero, tail);
    }

    if (motion_sign < 0.0f && start.v >= target_v - kEpsilon && start.a < -kEpsilon)
    {
        const float         t_zero     = -start.a / jm;
        const BoundaryState peak_state = EvaluateConstantJerk(start, jm, t_zero);
        const PrefixPlan    tail       = BuildVelocityClampPrefix(peak_state, target_v, am, jm);
        if (!tail.valid || tail.j_ramp <= 0.0f)
            return {};
        return MergePrefixWithLeadingJerk(start, jm, t_zero, tail);
    }

    const PrefixPlan offset_plan = BuildStopPrefix({ 0.0f, start.v - target_v, start.a }, am, jm);
    if (!offset_plan.valid)
        return {};
    return ShiftPrefixWithVelocityBias(offset_plan, start.x, target_v);
}

bool SCurveProfile::GetVelocityClampTarget(
        const BoundaryState& start, const float vm, const float jm, float* target_v)
{
    if (target_v == nullptr)
        return false;

    float motion_sign = Sign(start.v);
    if (motion_sign == 0.0f)
        motion_sign = Sign(start.a);
    if (motion_sign == 0.0f)
        return false;

    if (motion_sign > 0.0f)
    {
        if (start.v > vm + kEpsilon)
        {
            *target_v = vm;
            return true;
        }
        if (start.a > kEpsilon && start.v + kHalf * start.a * start.a / jm > vm + kEpsilon)
        {
            *target_v = vm;
            return true;
        }
        return false;
    }

    if (start.v < -vm - kEpsilon)
    {
        *target_v = -vm;
        return true;
    }
    if (start.a < -kEpsilon && start.v - kHalf * start.a * start.a / jm < -vm - kEpsilon)
    {
        *target_v = -vm;
        return true;
    }
    return false;
}

bool SCurveProfile::PrecheckCore(
        const BoundaryState& start, const BoundaryState& end, const float vm, const float jm)
{
    const float direction = end.x >= start.x ? 1.0f : -1.0f;

    const SidePrepare side_start = PrepareSide(start.v * direction, start.a * direction, vm, jm);
    const SidePrepare side_end   = PrepareSide(end.v * direction, -end.a * direction, vm, jm);
    if (!side_start.valid || !side_end.valid)
        return false;

    const float vp_min = fmaxf(fmaxf(side_start.vp_min, side_end.vp_min), 0.0f);
    return vp_min <= vm;
}

SCurveProfile::PrefixPlan SCurveProfile::TrimPrefix(const PrefixPlan& plan, const float cut_time)
{
    PrefixPlan trimmed{};
    trimmed.valid = false;
    if (!plan.valid)
        return trimmed;

    const float t_end = fminf(fmaxf(cut_time, 0.0f), plan.total_time);
    const float end_x = SamplePrefixX(plan, t_end);
    const float end_v = SamplePrefixV(plan, t_end);
    const float end_a = SamplePrefixA(plan, t_end);

    trimmed            = plan;
    trimmed.total_time = t_end;
    trimmed.end_x      = end_x;
    trimmed.end_v      = end_v;
    trimmed.end_a      = end_a;
    trimmed.valid      = IsFinite(t_end) && IsFinite(end_x) && IsFinite(end_v) && IsFinite(end_a);

    if (t_end <= kEpsilon)
    {
        trimmed.j_ramp     = 0.0f;
        trimmed.j_settle   = 0.0f;
        trimmed.t_ramp     = 0.0f;
        trimmed.t_hold     = 0.0f;
        trimmed.t_settle   = 0.0f;
        trimmed.x_ramp     = plan.x0;
        trimmed.v_ramp     = plan.v0;
        trimmed.a_ramp     = plan.a0;
        trimmed.x_hold     = plan.x0;
        trimmed.v_hold     = plan.v0;
        trimmed.a_hold     = plan.a0;
        return trimmed;
    }

    const float ramp_end = plan.t_ramp;
    const float hold_end = plan.t_ramp + plan.t_hold;
    if (t_end < ramp_end - kEpsilon)
    {
        trimmed.t_ramp   = t_end;
        trimmed.t_hold   = 0.0f;
        trimmed.t_settle = 0.0f;
        trimmed.j_settle = 0.0f;
        trimmed.x_ramp   = end_x;
        trimmed.v_ramp   = end_v;
        trimmed.a_ramp   = end_a;
        trimmed.x_hold   = end_x;
        trimmed.v_hold   = end_v;
        trimmed.a_hold   = end_a;
        return trimmed;
    }

    if (t_end < hold_end - kEpsilon)
    {
        trimmed.t_ramp   = plan.t_ramp;
        trimmed.t_hold   = t_end - plan.t_ramp;
        trimmed.t_settle = 0.0f;
        trimmed.j_settle = 0.0f;
        trimmed.x_hold   = end_x;
        trimmed.v_hold   = end_v;
        trimmed.a_hold   = end_a;
        return trimmed;
    }

    trimmed.t_ramp   = plan.t_ramp;
    trimmed.t_hold   = plan.t_hold;
    trimmed.t_settle = t_end - hold_end;
    return trimmed;
}

float SCurveProfile::SamplePrefixX(const PrefixPlan& plan, const float t)
{
    if (!plan.valid || t <= 0)
        return plan.x0;
    if (t < plan.t_ramp)
    {
        const float dt2 = t * t;
        const float dt3 = dt2 * t;
        return plan.x0 + plan.v0 * t + kHalf * plan.a0 * dt2 + kOneSixth * plan.j_ramp * dt3;
    }
    if (t < plan.t_ramp + plan.t_hold)
    {
        const float dt = t - plan.t_ramp;
        return plan.x_ramp + plan.v_ramp * dt + kHalf * plan.a_ramp * dt * dt;
    }
    if (t < plan.total_time)
    {
        const float dt  = t - plan.t_ramp - plan.t_hold;
        const float dt2 = dt * dt;
        const float dt3 = dt2 * dt;
        return plan.x_hold + plan.v_hold * dt + kHalf * plan.a_hold * dt2 + kOneSixth * plan.j_settle * dt3;
    }
    return plan.end_x;
}

float SCurveProfile::SamplePrefixV(const PrefixPlan& plan, const float t)
{
    if (!plan.valid || t <= 0)
        return plan.v0;
    if (t < plan.t_ramp)
        return plan.v0 + plan.a0 * t + kHalf * plan.j_ramp * t * t;
    if (t < plan.t_ramp + plan.t_hold)
    {
        const float dt = t - plan.t_ramp;
        return plan.v_ramp + plan.a_ramp * dt;
    }
    if (t < plan.total_time)
    {
        const float dt = t - plan.t_ramp - plan.t_hold;
        return plan.v_hold + plan.a_hold * dt + kHalf * plan.j_settle * dt * dt;
    }
    return plan.end_v;
}

float SCurveProfile::SamplePrefixA(const PrefixPlan& plan, const float t)
{
    if (!plan.valid || t <= 0)
        return plan.a0;
    if (t < plan.t_ramp)
        return plan.a0 + plan.j_ramp * t;
    if (t < plan.t_ramp + plan.t_hold)
        return plan.a_ramp;
    if (t < plan.total_time)
    {
        const float dt = t - plan.t_ramp - plan.t_hold;
        return plan.a_hold + plan.j_settle * dt;
    }
    return plan.end_a;
}

SCurveProfile::SidePrepare SCurveProfile::PrepareSide(
        const float v0, const float a0, const float vm, const float jm)
{
    SidePrepare ret{};
    ret.t_pre   = 0;
    ret.x_pre   = 0;
    ret.v_base  = 0;
    ret.t_shift = 0;
    ret.vp_min  = 0;
    ret.valid   = true;

    if (a0 < 0)
    {
        ret.v_base = v0 - kHalf * a0 * a0 / jm;
        if (fabsf(ret.v_base) > vm)
        {
            ret.valid = false;
            return ret;
        }
        ret.vp_min = ret.v_base;
        ret.t_pre  = -a0 / jm;
        ret.x_pre  = v0 * ret.t_pre + (1.0f / 3.0f) * a0 * ret.t_pre * ret.t_pre;
        return ret;
    }

    ret.vp_min = v0 + kHalf * a0 * a0 / jm;
    if (vm < ret.vp_min)
    {
        ret.valid = false;
        return ret;
    }
    ret.t_shift = a0 / jm;
    ret.v_base  = v0 - kHalf * a0 * ret.t_shift;
    if (ret.vp_min < 0)
        ret.vp_min = 0;
    return ret;
}

SCurveProfile::SolveStatus SCurveProfile::SolveCore(
        const Config& cfg, const BoundaryState& start, const BoundaryState& end, MotionCore* core) const
{
    if (core == nullptr)
        return SolveStatus::kInternalError;

    MotionCore out{};
    out.valid     = false;
    out.has_const = false;

    const float vm = fabsf(cfg.max_spd);
    const float am = fabsf(cfg.max_acc);
    const float jm = fabsf(cfg.max_jerk);

    const float dir = end.x >= start.x ? 1.0f : -1.0f;
    const float len = fabsf(end.x - start.x);

    out.direction = dir;
    out.xs        = start.x;
    out.xe        = end.x;
    out.vs        = start.v * dir;
    out.as        = start.a * dir;
    out.ve        = end.v * dir;
    out.ae        = end.a * dir;
    out.jm        = jm;
    out.vp        = 0;

    const SidePrepare side_start = PrepareSide(out.vs, out.as, vm, jm);
    const SidePrepare side_end   = PrepareSide(out.ve, -out.ae, vm, jm);
    if (!side_start.valid || !side_end.valid)
        return SolveStatus::kNeedsPrefixFallback;

    float vp_min = fmaxf(side_start.vp_min, side_end.vp_min);
    if (vp_min < 0)
        vp_min = 0;
    if (vm < vp_min)
        return SolveStatus::kNeedsPrefixFallback;

    out.t1_pre = side_start.t_pre;
    out.x1_pre = out.xs + dir * side_start.x_pre;
    out.ts1    = side_start.t_shift;
    out.t3_pre = side_end.t_pre;
    out.x3_pre = side_end.x_pre;
    out.ts3    = side_end.t_shift;

    const float len0 = len - side_start.x_pre - side_end.x_pre;
    if (len0 < -kDistanceTolerance)
        return SolveStatus::kNeedsPrefixFallback;

    const FastEvalConfig fast_eval_cfg{
        am,
        jm,
        am * am,
        am / jm,
        0.5f / am,
    };
    const FastEvalSide start_eval{ side_start.v_base, side_start.x_pre, side_start.t_shift };
    const FastEvalSide end_eval{ side_end.v_base, side_end.x_pre, side_end.t_shift };

    const FastEvalResult min_eval = EvaluateDistanceDelta(fast_eval_cfg, start_eval, end_eval, len, vp_min);
    if (min_eval.delta > kDistanceTolerance)
        return SolveStatus::kNeedsPrefixFallback;

    const FastEvalResult vm_eval = EvaluateDistanceDelta(fast_eval_cfg, start_eval, end_eval, len, vm);
    const float          x_const = -vm_eval.delta;
    if (x_const > 0)
    {
        out.process1.init(side_start.v_base, vm, am, jm);
        out.process3.init(side_end.v_base, vm, am, jm);
        out.xs1        = out.process1.getDistance(out.ts1);
        out.xs3        = out.process3.getDistance(out.ts3);
        out.has_const  = true;
        out.t1         = out.t1_pre + out.process1.getTotalTime() - out.ts1;
        out.t2         = out.t1 + x_const / vm;
        out.total_time = out.t2 + out.t3_pre + out.process3.getTotalTime() - out.ts3;
        out.x1         = out.xs + out.direction * vm_eval.dx1;
        out.vp         = vm;
        out.valid      = true;
        *core          = out;
        return SolveStatus::kSuccess;
    }

    float          l      = vp_min;
    float          r      = vm;
    FastEvalResult l_eval = min_eval;
    for (int iter = 0; iter < kPeakVelocityBisectIterations; ++iter)
    {
        const float mid = kHalf * (l + r);
        if (!(mid > l && mid < r))
            break;

        const FastEvalResult mid_eval = EvaluateDistanceDelta(fast_eval_cfg, start_eval, end_eval, len, mid);
        if (mid_eval.delta > 0)
        {
            r = mid;
        }
        else
        {
            l      = mid;
            l_eval = mid_eval;
            if (fabsf(mid_eval.delta) <= kDistanceTolerance)
                break;
        }
    }

    if (l_eval.delta > kDistanceTolerance)
        return SolveStatus::kNeedsPrefixFallback;

    const float vp = l;
    out.process1.init(side_start.v_base, vp, am, jm);
    out.process3.init(side_end.v_base, vp, am, jm);
    out.xs1 = out.process1.getDistance(out.ts1);
    out.xs3 = out.process3.getDistance(out.ts3);
    const float dx1     = side_start.x_pre + out.process1.getTotalDistance() - out.xs1;
    const float dx3     = side_end.x_pre + out.process3.getTotalDistance() - out.xs3;
    const float delta_d = dx1 + dx3 - len;
    // EvaluateDistanceDelta() and the full profile recomputation are not bit-identical.
    // Keep the same distance tolerance here to avoid fallback on accepted edge cases.
    if (delta_d > kDistanceTolerance)
        return SolveStatus::kNeedsPrefixFallback;

    const float residual_const = delta_d < 0.0f ? -delta_d : 0.0f;
    out.has_const             = residual_const > 0.0f;
    out.t1                    = out.t1_pre + out.process1.getTotalTime() - out.ts1;
    out.t2                    = out.t1 + (out.has_const ? residual_const / vp : 0.0f);
    out.total_time      = out.t2 + out.t3_pre + out.process3.getTotalTime() - out.ts3;
    out.x1              = out.xs + out.direction * dx1;
    out.vp              = vp;
    out.valid           = true;
    *core               = out;
    return SolveStatus::kSuccess;
}

bool SCurveProfile::TryVelocityClampRecovery(
        const Config& cfg, const BoundaryState& start, const BoundaryState& end, PrefixPlan* prefix,
        MotionCore* core) const
{
    if (prefix == nullptr || core == nullptr)
        return false;

    const float vm = fabsf(cfg.max_spd);
    const float am = fabsf(cfg.max_acc);
    const float jm = fabsf(cfg.max_jerk);

    float target_v = 0.0f;
    if (!GetVelocityClampTarget(start, vm, jm, &target_v))
        return false;

    const PrefixPlan clamp_prefix = BuildVelocityClampPrefix(start, target_v, am, jm);
    if (!clamp_prefix.valid)
        return false;

    const BoundaryState recovered{
        clamp_prefix.end_x,
        clamp_prefix.end_v,
        clamp_prefix.end_a,
    };
    MotionCore recovered_core{};
    if (SolveCore(cfg, recovered, end, &recovered_core) != SolveStatus::kSuccess)
        return false;

    *prefix = clamp_prefix;
    *core   = recovered_core;
    return true;
}

bool SCurveProfile::TryPrefixHandoff(
        const Config& cfg, const PrefixPlan& prefix_seed, const BoundaryState& end, PrefixPlan* prefix,
        MotionCore* core) const
{
    if (prefix == nullptr || core == nullptr || !prefix_seed.valid)
        return false;

    const float vm = fabsf(cfg.max_spd);
    const float jm = fabsf(cfg.max_jerk);

    float prev_t = 0.0f;
    for (int step = 1; step <= kRecoverySearchSteps; ++step)
    {
        const float t = prefix_seed.total_time * static_cast<float>(step) / static_cast<float>(kRecoverySearchSteps);
        const BoundaryState state{
            SamplePrefixX(prefix_seed, t),
            SamplePrefixV(prefix_seed, t),
            SamplePrefixA(prefix_seed, t),
        };
        if (!PrecheckCore(state, end, vm, jm))
        {
            prev_t = t;
            continue;
        }

        MotionCore candidate{};
        if (SolveCore(cfg, state, end, &candidate) != SolveStatus::kSuccess)
        {
            prev_t = t;
            continue;
        }

        float      lo      = prev_t;
        float      hi      = t;
        MotionCore hi_core = candidate;
        for (int iter = 0; iter < kRecoveryRefineSteps; ++iter)
        {
            const float         mid = kHalf * (lo + hi);
            const BoundaryState mid_state{
                SamplePrefixX(prefix_seed, mid),
                SamplePrefixV(prefix_seed, mid),
                SamplePrefixA(prefix_seed, mid),
            };
            if (!PrecheckCore(mid_state, end, vm, jm))
            {
                lo = mid;
                continue;
            }
            MotionCore mid_core{};
            if (SolveCore(cfg, mid_state, end, &mid_core) != SolveStatus::kSuccess)
            {
                lo = mid;
                continue;
            }
            hi      = mid;
            hi_core = mid_core;
        }

        *prefix = TrimPrefix(prefix_seed, hi);
        *core   = hi_core;
        return true;
    }

    return false;
}

float SCurveProfile::getReverseDistance(const MotionCore& core, const float tau) const
{
    if (tau <= 0)
        return 0;
    if (tau < core.t3_pre)
    {
        const float tau2 = tau * tau;
        const float tau3 = tau2 * tau;
        return core.ve * tau - kHalf * core.ae * tau2 + kOneSixth * core.jm * tau3;
    }
    return core.x3_pre + core.process3.getDistance(tau - core.t3_pre + core.ts3) - core.xs3;
}

float SCurveProfile::getReverseVelocity(const MotionCore& core, const float tau) const
{
    if (tau <= 0)
        return core.ve;
    if (tau < core.t3_pre)
        return core.ve - core.ae * tau + kHalf * core.jm * tau * tau;
    return core.process3.getVelocity(tau - core.t3_pre + core.ts3);
}

float SCurveProfile::getReverseAcceleration(const MotionCore& core, const float tau) const
{
    if (tau <= 0)
        return -core.ae;
    if (tau < core.t3_pre)
        return -core.ae + core.jm * tau;
    return core.process3.getAcceleration(tau - core.t3_pre + core.ts3);
}

float SCurveProfile::SampleCoreX(const MotionCore& core, const float t) const
{
    if (!core.valid)
        return 0;
    if (t <= 0)
        return core.xs;
    if (t < core.t1_pre)
    {
        const float t2 = t * t;
        const float t3 = t2 * t;
        return core.xs + core.direction * (core.vs * t + kHalf * core.as * t2 + kOneSixth * core.jm * t3);
    }
    if (t < core.t1)
        return core.x1_pre + core.direction * (core.process1.getDistance(t - core.t1_pre + core.ts1) - core.xs1);
    if (core.has_const && t < core.t2)
        return core.x1 + core.direction * core.vp * (t - core.t1);
    if (t < core.total_time)
        return core.xe - core.direction * getReverseDistance(core, core.total_time - t);
    return core.xe;
}

float SCurveProfile::SampleCoreV(const MotionCore& core, const float t) const
{
    if (!core.valid)
        return 0;
    if (t <= 0)
        return core.direction * core.vs;
    if (t < core.t1_pre)
        return core.direction * (core.vs + core.as * t + kHalf * core.jm * t * t);
    if (t < core.t1)
        return core.direction * core.process1.getVelocity(t - core.t1_pre + core.ts1);
    if (core.has_const && t < core.t2)
        return core.direction * core.vp;
    if (t < core.total_time)
        return core.direction * getReverseVelocity(core, core.total_time - t);
    return core.direction * core.ve;
}

float SCurveProfile::SampleCoreA(const MotionCore& core, const float t) const
{
    if (!core.valid)
        return 0;
    if (t <= 0)
        return core.direction * core.as;
    if (t < core.t1_pre)
        return core.direction * (core.as + core.jm * t);
    if (t < core.t1)
        return core.direction * core.process1.getAcceleration(t - core.t1_pre + core.ts1);
    if (core.has_const && t < core.t2)
        return 0;
    if (t < core.total_time)
        return -core.direction * getReverseAcceleration(core, core.total_time - t);
    return core.direction * core.ae;
}

float SCurveProfile::SampleSuffixX(const SuffixPlan& suffix, const float elapsed) const
{
    if (!suffix.valid)
        return main_core_.xe;
    const float tau = suffix.reverse_plan.total_time - elapsed;
    return suffix.start_x + suffix.reverse_plan.end_x - SamplePrefixX(suffix.reverse_plan, tau);
}

float SCurveProfile::SampleSuffixV(const SuffixPlan& suffix, const float elapsed) const
{
    if (!suffix.valid)
        return main_core_.direction * main_core_.ve;
    const float tau = suffix.reverse_plan.total_time - elapsed;
    return SamplePrefixV(suffix.reverse_plan, tau);
}

float SCurveProfile::SampleSuffixA(const SuffixPlan& suffix, const float elapsed) const
{
    if (!suffix.valid)
        return main_core_.direction * main_core_.ae;
    const float tau = suffix.reverse_plan.total_time - elapsed;
    return -SamplePrefixA(suffix.reverse_plan, tau);
}

SCurveProfile::SCurveProfile(
        const Config& cfg, float xs, float vs, float as, float xe, float ve, float ae)
{
    prefix_    = {};
    main_core_ = {};
    suffix_    = {};
    success_   = false;
    total_time_ = 0;
    failure_info_ = { cfg, xs, vs, as, xe, ve, ae, FailureCode::None };

    if (!IsFinite(cfg.max_spd) || !IsFinite(cfg.max_acc) || !IsFinite(cfg.max_jerk) || !IsFinite(xs) ||
        !IsFinite(xe) || !IsFinite(vs) || !IsFinite(as) || !IsFinite(ve) || !IsFinite(ae))
    {
        failure_info_.code = FailureCode::NonFiniteInput;
        return;
    }

    const float vm = fabsf(cfg.max_spd);
    const float am = fabsf(cfg.max_acc);
    const float jm = fabsf(cfg.max_jerk);
    if (vm <= 0 || am <= 0 || jm <= 0)
    {
        failure_info_.code = FailureCode::InvalidLimit;
        return;
    }
    if (fabsf(ve) > vm || fabsf(ae) > am)
    {
        failure_info_.code = FailureCode::InvalidEndState;
        return;
    }

    BoundaryState start{ xs, vs, as };
    BoundaryState end{ xe, ve, ae };

    const PrefixPlan reverse_end = BuildStopPrefix({ 0.0f, ve, -ae }, am, jm);
    BoundaryState    core_end    = end;
    if (reverse_end.valid && (fabsf(ve) > kEpsilon || fabsf(ae) > kEpsilon))
    {
        core_end.x = xe - reverse_end.end_x;
        core_end.v = reverse_end.end_v;
        core_end.a = -reverse_end.end_a;
        suffix_.reverse_plan = reverse_end;
        suffix_.start_x      = core_end.x;
        suffix_.valid        = true;
    }

    SolveStatus status = SolveCore(cfg, start, core_end, &main_core_);
    if (status != SolveStatus::kSuccess)
    {
        if (TryVelocityClampRecovery(cfg, start, core_end, &prefix_, &main_core_))
        {
            success_    = true;
            total_time_ = prefix_.total_time + main_core_.total_time +
                          (suffix_.valid ? suffix_.reverse_plan.total_time : 0.0f);
            return;
        }

        const PrefixPlan stop_prefix = BuildStopPrefix(start, am, jm);
        if (!stop_prefix.valid)
        {
            failure_info_.code = status == SolveStatus::kInternalError ? FailureCode::InternalError :
                                                                         FailureCode::StopPrefixFailed;
            return;
        }
        if (!TryPrefixHandoff(cfg, stop_prefix, core_end, &prefix_, &main_core_))
        {
            failure_info_.code = FailureCode::PrefixHandoffFailed;
            return;
        }
    }

    success_    = true;
    total_time_ = prefix_.total_time + main_core_.total_time +
                  (suffix_.valid ? suffix_.reverse_plan.total_time : 0.0f);
}

float SCurveProfile::CalcX(const float t) const
{
    if (!success_)
        return 0;
    if (t <= 0)
        return prefix_.valid ? prefix_.x0 : main_core_.xs;

    if (prefix_.valid && t < prefix_.total_time)
        return SamplePrefixX(prefix_, t);

    const float core_start = prefix_.valid ? prefix_.total_time : 0.0f;
    const float core_end   = core_start + main_core_.total_time;
    if (t < core_end)
        return SampleCoreX(main_core_, t - core_start);

    if (suffix_.valid)
        return SampleSuffixX(suffix_, t - core_end);

    return main_core_.xe;
}

float SCurveProfile::CalcV(const float t) const
{
    if (!success_)
        return 0;
    if (t <= 0)
        return prefix_.valid ? prefix_.v0 : main_core_.direction * main_core_.vs;

    if (prefix_.valid && t < prefix_.total_time)
        return SamplePrefixV(prefix_, t);

    const float core_start = prefix_.valid ? prefix_.total_time : 0.0f;
    const float core_end   = core_start + main_core_.total_time;
    if (t < core_end)
        return SampleCoreV(main_core_, t - core_start);

    if (suffix_.valid)
        return SampleSuffixV(suffix_, t - core_end);

    return main_core_.direction * main_core_.ve;
}

float SCurveProfile::CalcA(const float t) const
{
    if (!success_)
        return 0;
    if (t <= 0)
        return prefix_.valid ? prefix_.a0 : main_core_.direction * main_core_.as;

    if (prefix_.valid && t < prefix_.total_time)
        return SamplePrefixA(prefix_, t);

    const float core_start = prefix_.valid ? prefix_.total_time : 0.0f;
    const float core_end   = core_start + main_core_.total_time;
    if (t < core_end)
        return SampleCoreA(main_core_, t - core_start);

    if (suffix_.valid)
        return SampleSuffixA(suffix_, t - core_end);

    return main_core_.direction * main_core_.ae;
}
} // namespace velocity_profile
