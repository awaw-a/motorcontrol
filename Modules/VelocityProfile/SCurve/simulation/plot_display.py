# -*- coding: utf-8 -*-
from s_curve import SCurve

import matplotlib.pyplot as plt
import numpy as np


def plot_scurve(xs, xe, vs, as_, vm, am, jm, ve=0.0, ae=0.0, dt=0.001):
    s = SCurve()
    if s.init(xs, xe, vs, as_, vm, am, jm, ve, ae) == s.S_CURVE_FAILED:
        print("S curve init failed")
        return

    total_time = s.total_time
    if total_time <= 0.0:
        t = np.array([0.0])
    else:
        t = np.arange(0, total_time + dt, dt)

    x = np.array([s.calc_x(ti) for ti in t])
    v = np.array([s.calc_v(ti) for ti in t])
    a = np.array([s.calc_a(ti) for ti in t])
    j = np.gradient(a, t, edge_order=1) if len(t) >= 2 else np.zeros_like(a)

    markers = []
    if s.prefix.valid and s.prefix.total_time > 0.0:
        markers.append(s.prefix.total_time)
    if s.main_core.valid:
        markers.append((s.prefix.total_time if s.prefix.valid else 0.0) + s.main_core.total_time)

    print("===== SCurve quick check =====")
    print(f"total_time: {total_time:.6f}s")
    if s.main_core.valid:
        print(f"peak_v: {s.main_core.vp:.6f}, has_const: {s.main_core.has_const}")
    print(f"debug: {s.debug}")
    if s.prefix.valid:
        print(
            "prefix:"
            f" T={s.prefix.total_time:.6f}s"
            f" end=({s.prefix.end_x:.6f}, {s.prefix.end_v:.6f}, {s.prefix.end_a:.6f})"
        )
    if s.main_core.valid:
        print(
            "core:"
            f" T={s.main_core.total_time:.6f}s"
            f" x0={s.main_core.xs:.6f}"
            f" xe={s.main_core.xe:.6f}"
            f" dir={s.main_core.direction:.0f}"
        )
    if s.suffix.valid:
        rp = s.suffix.reverse_plan
        print(
            "suffix(reverse stop):"
            f" T={rp.total_time:.6f}s"
            f" end=({rp.end_x:.6f}, {rp.end_v:.6f}, {rp.end_a:.6f})"
        )
    print(f"x(0)/x(T): {x[0]:.6f} / {x[-1]:.6f} (target {xs:.6f} / {xe:.6f})")
    print(f"v(0)/v(T): {v[0]:.6f} / {v[-1]:.6f} (target {vs:.6f} / {ve:.6f})")
    print(f"a(0)/a(T): {a[0]:.6f} / {a[-1]:.6f} (target {as_:.6f} / {ae:.6f})")
    print(f"max|v|={np.max(np.abs(v)):.6f} <= vm({vm:.6f})")
    print(f"max|a|={np.max(np.abs(a)):.6f} <= am({am:.6f})")
    print(f"max|j|={np.max(np.abs(j)):.6f} <= jm({jm:.6f}) [numerical]")

    plt.figure()
    plt.plot(t, x)
    for marker in markers:
        plt.axvline(marker, linestyle="--", linewidth=0.8, color="tab:red")
    plt.title("x(t)")
    plt.xlabel("t")
    plt.ylabel("x")
    plt.grid()

    plt.figure()
    plt.plot(t, v)
    for marker in markers:
        plt.axvline(marker, linestyle="--", linewidth=0.8, color="tab:red")
    plt.title("v(t)")
    plt.xlabel("t")
    plt.ylabel("v")
    plt.grid()

    plt.figure()
    plt.plot(t, a)
    for marker in markers:
        plt.axvline(marker, linestyle="--", linewidth=0.8, color="tab:red")
    plt.title("a(t)")
    plt.xlabel("t")
    plt.ylabel("a")
    plt.grid()

    plt.figure()
    plt.plot(t, j)
    for marker in markers:
        plt.axvline(marker, linestyle="--", linewidth=0.8, color="tab:red")
    plt.title("j(t)")
    plt.xlabel("t")
    plt.ylabel("j")
    plt.grid()

    plt.show()


if __name__ == "__main__":
    plot_scurve(
        xs=0.0,
        xe=1.0,
        vs=4.0,
        as_=2.5,
        vm=1.5,
        am=1.0,
        jm=2.0,
        ve=0.0,
        ae=0.0,
    )
