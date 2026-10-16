/**
 * @file    s_curve.hpp
 * @author  syhanjin LIJunHong659
 * @date    2026-01-28
 */
#ifndef S_CURVE_HPP
#define S_CURVE_HPP

#include <cstdint>

#include "IVelocityProfile.hpp"

#ifndef S_CURVE_MAX_BS_ERROR
#    define S_CURVE_MAX_BS_ERROR (0.001f)
#endif

namespace velocity_profile
{

class SCurveProfile final : public IVelocityProfile
{
public:
    struct Config
    {
        float max_spd;
        float max_acc;
        float max_jerk;

        constexpr Config operator*(const float ratio) const
        {
            return { max_spd * ratio, max_acc * ratio, max_jerk * ratio };
        }
        constexpr Config operator/(const float factor) const
        {
            return { max_spd / factor, max_acc / factor, max_jerk / factor };
        }
    };

    enum class FailureCode : uint8_t
    {
        None                        = 0,
        NonFiniteInput              = 1,
        InvalidLimit                = 2,
        InvalidEndState             = 3,
        SolveCoreFailed             = 4,
        VelocityClampRecoveryFailed = 5,
        StopPrefixFailed            = 6,
        PrefixHandoffFailed         = 7,
        InternalError               = 8,
    };

    struct FailureInfo
    {
        Config      cfg{};
        float       xs{};
        float       vs{};
        float       as{};
        float       xe{};
        float       ve{};
        float       ae{};
        FailureCode code{ FailureCode::None };
    };

    SCurveProfile(
            const Config& cfg, float xs, float vs, float as, float xe, float ve = 0, float ae = 0);

    [[nodiscard]] float CalcX(float t) const override;
    [[nodiscard]] float CalcV(float t) const override;
    [[nodiscard]] float CalcA(float t) const override;
    [[nodiscard]] float getTotalTime() const override { return total_time_; }
    [[nodiscard]] bool success() const override { return success_; }
    [[nodiscard]] const FailureInfo& failureInfo() const { return failure_info_; }

private:
    class SCurveAccel
    {
    public:
        SCurveAccel();
        void init(float vs, float vp, float am, float jm);

        [[nodiscard]] float getDistance(float t) const;
        [[nodiscard]] float getVelocity(float t) const;
        [[nodiscard]] float getAcceleration(float t) const;
        [[nodiscard]] float getTotalDistance() const { return total_distance_; }
        [[nodiscard]] float getTotalTime() const { return total_time_; }

    private:
        bool  has_uniform_;
        float vs_;
        float jm_;

        float total_time_;
        float total_distance_;

        float t1_;
        float x1_;
        float v1_;
        float t2_;

        float ap_;
        float vp_;
    };

    enum class SolveStatus : uint8_t
    {
        kSuccess,
        kNeedsPrefixFallback,
        kInternalError,
    };

    struct BoundaryState
    {
        float x;
        float v;
        float a;
    };

    struct SidePrepare
    {
        float t_pre;
        float x_pre;
        float v_base;
        float t_shift;
        float vp_min;
        bool  valid;
    };

    struct PrefixPlan
    {
        float x0;
        float v0;
        float a0;
        float j_ramp;
        float j_settle;

        float t_ramp;
        float t_hold;
        float t_settle;

        float x_ramp;
        float v_ramp;
        float a_ramp;

        float x_hold;
        float v_hold;
        float a_hold;

        float total_time;
        float end_x;
        float end_v;
        float end_a;

        bool valid;
    };

    struct MotionCore
    {
        bool  valid;
        bool  has_const;
        float direction;
        float xs;
        float xe;
        float vs;
        float as;
        float ve;
        float ae;
        float jm;
        float vp;

        float t1_pre;
        float x1_pre;
        float ts1;
        float xs1;

        float t3_pre;
        float x3_pre;
        float ts3;
        float xs3;

        float t1;
        float t2;
        float x1;
        float total_time;

        SCurveAccel process1;
        SCurveAccel process3;
    };

    struct SuffixPlan
    {
        PrefixPlan reverse_plan;
        float      start_x;
        bool       valid;
    };

    [[nodiscard]] static bool IsFinite(float value);
    [[nodiscard]] static float Sign(float value);

    [[nodiscard]] static BoundaryState EvaluateConstantJerk(
            const BoundaryState& state, float jerk, float dt);
    [[nodiscard]] static PrefixPlan BuildStopPrefix(const BoundaryState& start, float am, float jm);
    [[nodiscard]] static PrefixPlan ShiftPrefixWithVelocityBias(
            const PrefixPlan& plan, float x_origin, float velocity_bias);
    [[nodiscard]] static PrefixPlan MergePrefixWithLeadingJerk(
            const BoundaryState& start, float jerk, float lead_time, const PrefixPlan& tail);
    [[nodiscard]] static PrefixPlan BuildVelocityClampPrefix(
            const BoundaryState& start, float target_v, float am, float jm);
    [[nodiscard]] static bool GetVelocityClampTarget(
            const BoundaryState& start, float vm, float jm, float* target_v);
    [[nodiscard]] static bool       PrecheckCore(
            const BoundaryState& start, const BoundaryState& end, float vm, float jm);
    [[nodiscard]] static PrefixPlan TrimPrefix(const PrefixPlan& plan, float cut_time);
    [[nodiscard]] static float        SamplePrefixX(const PrefixPlan& plan, float t);
    [[nodiscard]] static float        SamplePrefixV(const PrefixPlan& plan, float t);
    [[nodiscard]] static float        SamplePrefixA(const PrefixPlan& plan, float t);

    [[nodiscard]] static SidePrepare PrepareSide(float v0, float a0, float vm, float jm);

    [[nodiscard]] SolveStatus SolveCore(
            const Config& cfg, const BoundaryState& start, const BoundaryState& end, MotionCore* core) const;
    [[nodiscard]] bool TryVelocityClampRecovery(
            const Config& cfg, const BoundaryState& start, const BoundaryState& end, PrefixPlan* prefix,
            MotionCore* core) const;
    [[nodiscard]] bool TryPrefixHandoff(
            const Config& cfg, const PrefixPlan& prefix_seed, const BoundaryState& end, PrefixPlan* prefix,
            MotionCore* core) const;

    [[nodiscard]] float getReverseDistance(const MotionCore& core, float tau) const;
    [[nodiscard]] float getReverseVelocity(const MotionCore& core, float tau) const;
    [[nodiscard]] float getReverseAcceleration(const MotionCore& core, float tau) const;
    [[nodiscard]] float SampleSuffixX(const SuffixPlan& suffix, float elapsed) const;
    [[nodiscard]] float SampleSuffixV(const SuffixPlan& suffix, float elapsed) const;
    [[nodiscard]] float SampleSuffixA(const SuffixPlan& suffix, float elapsed) const;

    [[nodiscard]] float SampleCoreX(const MotionCore& core, float t) const;
    [[nodiscard]] float SampleCoreV(const MotionCore& core, float t) const;
    [[nodiscard]] float SampleCoreA(const MotionCore& core, float t) const;

    bool       success_{ false };
    FailureInfo failure_info_{};
    PrefixPlan prefix_{};
    MotionCore main_core_{};
    SuffixPlan suffix_{};
    float      total_time_{ 0.0f };
};

} // namespace velocity_profile

#endif // S_CURVE_HPP
