import math
from dataclasses import dataclass

from s_curve import SCurve


@dataclass
class Posture:
    x: float
    y: float
    yaw: float

    def __add__(self, other: 'Posture') -> 'Posture':
        return Posture(self.x + other.x, self.y + other.y, self.yaw + other.yaw, )


@dataclass
class Velocity:
    vx: float
    vy: float
    wz: float

    def __add__(self, other: 'Velocity') -> 'Velocity':
        return Velocity(self.vx + other.vx, self.vy + other.vy, self.wz + other.wz, )


@dataclass
class Acceleration:
    ax: float
    ay: float
    ayaw: float

    def __add__(self, other: 'Acceleration') -> 'Acceleration':
        return Acceleration(self.ax + other.ax, self.ay + other.ay, self.ayaw + other.ayaw)

@dataclass
class Limit:
    v_max: float
    a_max: float
    j_max: float

def chassis_plot(start: Posture,
                 end: Posture,
                 start_velocity: Velocity,
                 start_accel: Acceleration,
                 x_limit: Limit,
                 y_limit: Limit,
                 yaw_limit: Limit,
                 end_velocity: Velocity = None,
                 end_accel: Acceleration = None) -> None:
    sx = SCurve()
    if end_velocity is None:
        end_velocity = Velocity(vx=0.0, vy=0.0, wz=0.0)
    if end_accel is None:
        end_accel = Acceleration(ax=0.0, ay=0.0, ayaw=0.0)

    ret = sx.init(start.x, end.x, start_velocity.vx, start_accel.ax,
                  x_limit.v_max, x_limit.a_max, x_limit.j_max,
                  end_velocity.vx, end_accel.ax)
    if ret == sx.S_CURVE_FAILED:
        print("sx init failed")
        return

    sy = SCurve()
    ret = sy.init(start.y, end.y, start_velocity.vy, start_accel.ay,
                  y_limit.v_max, y_limit.a_max, y_limit.j_max,
                  end_velocity.vy, end_accel.ay)
    if ret == sy.S_CURVE_FAILED:
        print("sy init failed")
        return

    syaw = SCurve()
    ret = syaw.init(start.yaw, end.yaw, start_velocity.wz, start_accel.ayaw,
                    yaw_limit.v_max, yaw_limit.a_max, yaw_limit.j_max,
                    end_velocity.wz, end_accel.ayaw)
    if ret == syaw.S_CURVE_FAILED:
        print("syaw init failed")
        return

    print("===== Chassis axis debug =====")
    print(f"x-axis: {sx.debug}")
    print(f"y-axis: {sy.debug}")
    print(f"yaw-axis: {syaw.debug}")

    def p_in_w(t) -> Posture:
        return start + Posture(
            x=sx.calc_x(t),
            y=sy.calc_x(t),
            yaw=syaw.calc_x(t)
        )

    def v_in_w(t) -> Velocity:
        return Velocity(
            sx.calc_v(t),
            sy.calc_v(t),
            syaw.calc_v(t)
        )

    def a_in_w(t) -> Acceleration:
        return Acceleration(
            sx.calc_a(t),
            sy.calc_a(t),
            syaw.calc_a(t)
        )

    def v_in_b(t) -> Velocity:
        p = p_in_w(t)
        theta = math.radians(p.yaw)
        v = v_in_w(t)
        return Velocity(
            vx=v.vx * math.cos(-theta) - v.vy * math.sin(-theta),
            vy=v.vx * math.sin(-theta) + v.vy * math.cos(-theta),
            wz=v.wz
        )

    def a_in_b(t) -> Acceleration:
        p = p_in_w(t)
        theta = math.radians(p.yaw)
        v = v_in_w(t)
        omega = math.radians(v.wz)
        a = a_in_w(t)
        return Acceleration(
            a.ax * math.cos(theta) + a.ay * math.sin(theta) + omega * (
                        -v.vx * math.sin(theta) + v.vy * math.cos(theta)),
            -a.ax * math.sin(theta) + a.ay * math.cos(theta) + omega * (
                        -v.vx * math.cos(theta) - v.vy * math.sin(theta)),
            a.ayaw
        )

    total_time = max(sx.total_time, sy.total_time, syaw.total_time)
    dt = 0.001

    ts = [i * dt for i in range(int(total_time / dt) + 1)]

    # world posture
    px, py, pyaw = [], [], []
    for t in ts:
        p = p_in_w(t)
        px.append(p.x)
        py.append(p.y)
        pyaw.append(p.yaw)

    # world velocity
    vx_w, vy_w, wz_w = [], [], []
    for t in ts:
        v = v_in_w(t)
        vx_w.append(v.vx)
        vy_w.append(v.vy)
        wz_w.append(v.wz)

    # world acceleration
    ax_w, ay_w, ayaw_w = [], [], []
    for t in ts:
        a = a_in_w(t)
        ax_w.append(a.ax)
        ay_w.append(a.ay)
        ayaw_w.append(a.ayaw)

    # body velocity
    vx_b, vy_b, wz_b = [], [], []
    for t in ts:
        v = v_in_b(t)
        vx_b.append(v.vx)
        vy_b.append(v.vy)
        wz_b.append(v.wz)

    # body acceleration
    ax_b, ay_b, ayaw_b = [], [], []
    for t in ts:
        a = a_in_b(t)
        ax_b.append(a.ax)
        ay_b.append(a.ay)
        ayaw_b.append(a.ayaw)

    import matplotlib.pyplot as plt

    # 1 p_in_w
    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(ts, px)
    plt.ylabel("x")

    plt.subplot(3, 1, 2)
    plt.plot(ts, py)
    plt.ylabel("y")

    plt.subplot(3, 1, 3)
    plt.plot(ts, pyaw)
    plt.ylabel("yaw")
    plt.xlabel("t")

    plt.suptitle("p_in_w")

    # 2 v_in_w
    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(ts, vx_w)
    plt.ylabel("vx")

    plt.subplot(3, 1, 2)
    plt.plot(ts, vy_w)
    plt.ylabel("vy")

    plt.subplot(3, 1, 3)
    plt.plot(ts, wz_w)
    plt.ylabel("wz")
    plt.xlabel("t")

    plt.suptitle("v_in_w")

    # 3 a_in_w
    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(ts, ax_w)
    plt.ylabel("ax")

    plt.subplot(3, 1, 2)
    plt.plot(ts, ay_w)
    plt.ylabel("ay")

    plt.subplot(3, 1, 3)
    plt.plot(ts, ayaw_w)
    plt.ylabel("ayaw")
    plt.xlabel("t")

    plt.suptitle("a_in_w")

    # 4 v_in_b
    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(ts, vx_b)
    plt.ylabel("vx_b")

    plt.subplot(3, 1, 2)
    plt.plot(ts, vy_b)
    plt.ylabel("vy_b")

    plt.subplot(3, 1, 3)
    plt.plot(ts, wz_b)
    plt.ylabel("wz_b")
    plt.xlabel("t")

    plt.suptitle("v_in_b")

    # 5 a_in_b
    plt.figure()
    plt.subplot(3, 1, 1)
    plt.plot(ts, ax_b)
    plt.ylabel("ax_b")

    plt.subplot(3, 1, 2)
    plt.plot(ts, ay_b)
    plt.ylabel("ay_b")

    plt.subplot(3, 1, 3)
    plt.plot(ts, ayaw_b)
    plt.ylabel("ayaw_b")
    plt.xlabel("t")

    plt.suptitle("a_in_b")
    plt.show()

if __name__ == "__main__":
    chassis_plot(
        start=Posture(x=-0.00516828925553639, y=0, yaw=0),
        end=Posture(x=-0.0660718854882826, y=0, yaw=0),
        start_velocity=Velocity(vx=0.25297227494895586, vy=0.0, wz=0),
        start_accel=Acceleration(ax=-0.20557271586122505, ay=0, ayaw=0),
        end_velocity=Velocity(vx=0.07192315316602482, vy=0, wz=0.0),
        end_accel=Acceleration(ax=-0.6722352158596954, ay=0.0, ayaw=0.0),
        x_limit=Limit(v_max=0.5412215891127709, a_max=0.9325631134045481, j_max=3.5764751321784343),
        y_limit=Limit(v_max=1, a_max=1, j_max=2),
        yaw_limit=Limit(v_max=180, a_max=60, j_max=360)
    )
