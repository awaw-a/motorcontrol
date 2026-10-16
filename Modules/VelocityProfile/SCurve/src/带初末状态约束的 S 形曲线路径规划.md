# 带初末状态约束的 S 形曲线路径规划

本文对应 `SCurve/src/s_curve.hpp` 与 `SCurve/src/s_curve.cpp` 的当前实现，目标是说明这套 S 曲线规划器如何在满足速度、加速度、加加速度约束的前提下，同时兼容：

- 任意起点速度 $v_s$、起点加速度 $a_s$；
- 任意终点速度 $v_e$、终点加速度 $a_e$；
- 可选的匀速段；
- 仅靠加速段与减速段拼接的短距离运动。

文中的数学符号尽量与代码成员保持一一对应，便于对照阅读实现。

## 1. 问题定义

我们希望在一维区间 $[x_s, x_e]$ 上构造一条轨迹 $x(t)$，使其满足：

$$
|v(t)| \le v_m,\qquad |a(t)| \le a_m,\qquad |j(t)| \le j_m
$$

其中

$$
v(t)=\dot x(t),\qquad a(t)=\ddot x(t),\qquad j(t)=\dddot x(t)
$$

边界条件为：

$$
x(0)=x_s,\qquad v(0)=v_s,\qquad a(0)=a_s
$$

$$
x(T)=x_e,\qquad v(T)=v_e,\qquad a(T)=a_e
$$

代码采用的是“先求可行的最大峰值速度，再决定是否插入匀速段”的构造策略。在当前分段模型下，它等价于求解一条满足约束的最快轨迹。

## 2. 方向归一化

实现中的第一步不是直接在原坐标系里求解，而是把运动统一折算为“正向问题”。

定义方向因子

$$
d=\operatorname{sign}(x_e-x_s)\in\{+1,-1\}
$$

以及总路程

$$
L=|x_e-x_s|
$$

再把速度和加速度映射到归一化坐标系：

$$
\bar v_s=d\,v_s,\quad \bar a_s=d\,a_s,\quad \bar v_e=d\,v_e,\quad \bar a_e=d\,a_e
$$

这样就可以把问题统一写成：

- 从 $\bar x(0)=0$ 运动到 $\bar x(T)=L$；
- 中间速度尽量保持非负；
- 最后再通过
  $$
  x(t)=x_s+d\,\bar x(t),\qquad v(t)=d\,\bar v(t),\qquad a(t)=d\,\bar a(t)
  $$
  恢复原始方向。

这正是代码里 `direction_`、`xs_`、`xe_` 以及对 `vs/as/ve/ae` 统一乘方向的原因。

## 3. 标准单边加速器 `SCurveAccel`

整个求解器的基础模块是 `SCurveAccel`。它只处理一种标准问题：

> 已知起始速度 $v_b$，起始加速度为 0；在约束 $a_m, j_m$ 下，把速度单调提升到峰值 $v_p$，并在结束时回到加速度 0。

也就是说，`SCurveAccel` 不直接处理任意初始加速度，也不直接处理终点边界；它只负责一个“标准单边过程”。

为方便描述，记该过程的位移、速度、加速度函数分别为：

$$
\mathcal S(t; v_b, v_p),\qquad \mathcal V(t; v_b, v_p),\qquad \mathcal A(t; v_b, v_p)
$$

总时长与总位移分别记为：

$$
T_{\mathrm{acc}}(v_b,v_p),\qquad S_{\mathrm{acc}}(v_b,v_p)
$$

### 3.1 梯形加速度曲线

当增速足够大、能够触及最大加速度时，有：

$$
j_m(v_p-v_b)>a_m^2
$$

此时过程分为三段：

1. `jerk` 为 $+j_m$ 的加加速段；
2. 加速度保持 $a_m$ 的匀加速段；
3. `jerk` 为 $-j_m$ 的减加速段。

定义

$$
t_j=\frac{a_m}{j_m}
$$

在代码中，对应的阶段边界写成：

$$
t_1=t_j,\qquad t_2=\frac{v_p-v_b}{a_m},\qquad T_{\mathrm{acc}}=t_1+t_2
$$

注意这里的 $t_2$ 不是“第二段时长”，而是“第二段结束时刻”；因此匀加速段的实际长度是 $t_2-t_1$。

第一阶段结束时的速度、位移为：

$$
v_1=v_b+\frac{1}{2}a_m t_1
$$

$$
x_1=v_b t_1+\frac{1}{6}j_m t_1^3
    =v_b t_1+\frac{1}{6}a_m t_1^2
$$

于是分段表达式为：

$$
\mathcal A(t)=
\begin{cases}
j_m t, & 0\le t<t_1 \\
a_m, & t_1\le t<t_2 \\
j_m(T_{\mathrm{acc}}-t), & t_2\le t<T_{\mathrm{acc}}
\end{cases}
$$

$$
\mathcal V(t)=
\begin{cases}
v_b+\dfrac{1}{2}j_m t^2, & 0\le t<t_1 \\
v_1+a_m(t-t_1), & t_1\le t<t_2 \\
v_p-\dfrac{1}{2}j_m(T_{\mathrm{acc}}-t)^2, & t_2\le t<T_{\mathrm{acc}}
\end{cases}
$$

$$
\mathcal S(t)=
\begin{cases}
v_b t+\dfrac{1}{6}j_m t^3, & 0\le t<t_1 \\
x_1+v_1(t-t_1)+\dfrac{1}{2}a_m(t-t_1)^2, & t_1\le t<t_2 \\
S_{\mathrm{acc}}-v_p(T_{\mathrm{acc}}-t)+\dfrac{1}{6}j_m(T_{\mathrm{acc}}-t)^3, & t_2\le t<T_{\mathrm{acc}}
\end{cases}
$$

总位移可写为：

$$
S_{\mathrm{acc}}
=\frac{v_p^2-v_b^2}{2a_m}+\frac{1}{2}(v_b+v_p)t_1
$$

这正对应 `SCurveAccel::init()` 中 `has_uniform_ == true` 时的参数构造。

### 3.2 三角加速度曲线

当速度增量不足以触及最大加速度时，有：

$$
j_m(v_p-v_b)\le a_m^2
$$

此时最大加速度不再是 $a_m$，而是由速度增量反推出来的峰值：

$$
a_p=\sqrt{j_m(v_p-v_b)}
$$

于是只有两段：

1. `jerk` 为 $+j_m$ 的加加速段；
2. `jerk` 为 $-j_m$ 的减加速段。

此时

$$
t_1=t_2=\frac{a_p}{j_m},\qquad T_{\mathrm{acc}}=2t_1
$$

第一阶段末端状态为：

$$
v_1=v_b+\frac{1}{2}\frac{a_p^2}{j_m},\qquad
x_1=v_b t_1+\frac{1}{6}a_p t_1^2
$$

总位移为：

$$
S_{\mathrm{acc}}=(v_b+v_p)t_1
$$

从实现上看，它与上一节的三段公式是“同一套骨架”，只是中间匀加速段长度退化为 0，且 $a_m$ 换成 $a_p$。

## 4. 任意边界的单侧规整 `prepareSide`

现实问题里，起点和终点的加速度并不一定为 0，因此不能直接交给 `SCurveAccel`。  
所以代码引入了一个中间步骤：把任意单侧边界 $(v_0,a_0)$ 规整成标准单边加速器可处理的形式。

对每一侧，`prepareSide(v_0,a_0)` 输出 5 个量：

- $t_{\mathrm{pre}}$：显式预处理时长；
- $x_{\mathrm{pre}}$：显式预处理位移；
- $v_{\mathrm{base}}$：进入标准加速器时的等效基线速度；
- $t_{\mathrm{shift}}$：在标准加速器上的时间偏移；
- $v_{p,\min}$：该侧允许的峰值速度下界。

### 4.1 情形 A：当前加速度与目标方向相反

若

$$
a_0<0
$$

说明当前加速度指向“减速方向”，必须先用正 `jerk` 把加速度抬回 0，再进入主加速过程。

这一小段显式预处理满足：

$$
a(t)=a_0+j_m t
$$

$$
v(t)=v_0+a_0 t+\frac{1}{2}j_m t^2
$$

$$
x(t)=v_0 t+\frac{1}{2}a_0 t^2+\frac{1}{6}j_m t^3
$$

当加速度回到 0 时：

$$
t_{\mathrm{pre}}=-\frac{a_0}{j_m}
$$

$$
v_{\mathrm{base}}
=v_0+a_0 t_{\mathrm{pre}}+\frac{1}{2}j_m t_{\mathrm{pre}}^2
=v_0-\frac{a_0^2}{2j_m}
$$

$$
x_{\mathrm{pre}}
=v_0 t_{\mathrm{pre}}+\frac{1}{2}a_0 t_{\mathrm{pre}}^2+\frac{1}{6}j_m t_{\mathrm{pre}}^3
=v_0 t_{\mathrm{pre}}+\frac{1}{3}a_0 t_{\mathrm{pre}}^2
$$

此时没有时间平移，因此：

$$
t_{\mathrm{shift}}=0,\qquad v_{p,\min}=v_{\mathrm{base}}
$$

如果 $v_{\mathrm{base}}$ 已经超过速度上限 $v_m$，那么这一侧立即判为无解。

### 4.2 情形 B：当前加速度与目标方向同向

若

$$
a_0\ge 0
$$

说明当前状态本质上已经处于某个标准 S 曲线的“前半段”中，无需显式插入预处理，只需把这段状态看作标准过程在某个时刻之后的截面。

因为标准加速器第一段满足

$$
\mathcal A(t)=j_m t
$$

所以只需选择

$$
t_{\mathrm{shift}}=\frac{a_0}{j_m}
$$

即可让该标准过程在 $t=t_{\mathrm{shift}}$ 时具有同样的加速度。

再利用速度匹配：

$$
v_0=v_{\mathrm{base}}+\frac{1}{2}j_m t_{\mathrm{shift}}^2
$$

得到

$$
v_{\mathrm{base}}
=v_0-\frac{1}{2}j_m t_{\mathrm{shift}}^2
=v_0-\frac{a_0^2}{2j_m}
$$

此时不需要显式位移补偿：

$$
t_{\mathrm{pre}}=0,\qquad x_{\mathrm{pre}}=0
$$

但不是任意 $v_p$ 都能支持这个时间平移。为了让 $t_{\mathrm{shift}}$ 仍然落在标准过程的前半段，必须满足：

$$
v_p \ge v_0+\frac{a_0^2}{2j_m}
$$

于是该侧峰值速度下界为：

$$
v_{p,\min}=v_0+\frac{a_0^2}{2j_m}
$$

若 $v_m < v_{p,\min}$，则该侧也无解。

## 5. 终点约束为什么可以用时间反演处理

起点的处理方式已经清楚了，但终点还有 $(v_e,a_e)$ 约束，不能再简单地把末段写成“从 0 加速到 $v_p$ 的镜像”。

代码的核心技巧是：把末端过程转写成一个时间反演问题。

在归一化坐标系中，定义反演时间

$$
\tau=T-t
$$

再定义反向累计位移

$$
X_r(\tau)=L-\bar x(T-\tau)
$$

对它求导可得：

$$
V_r(\tau)=\frac{dX_r}{d\tau}=\bar v(T-\tau)
$$

$$
A_r(\tau)=\frac{dV_r}{d\tau}=-\bar a(T-\tau)
$$

因此在 $\tau=0$ 处有：

$$
V_r(0)=\bar v_e,\qquad A_r(0)=-\bar a_e
$$

这就把“终点边界”变成了“反向时间中的起点边界”，可以直接交给与起点完全相同的 `prepareSide` 逻辑处理。

这也是代码中：

- 起点调用 `prepareSide(vs, as)`；
- 终点调用 `prepareSide(ve, -ae)`；

的数学来源。

## 6. 单侧消耗位移与峰值速度求解

经过 `prepareSide` 之后，起点侧与终点反演侧都被规整成了：

- 一段显式预处理；
- 一段标准单边加速器；
- 其中标准加速器可能还要跳过前面的 $t_{\mathrm{shift}}$。

因此，对任意一侧，其总消耗位移可写为：

$$
D(v_p)
=x_{\mathrm{pre}}
 + S_{\mathrm{acc}}(v_{\mathrm{base}},v_p)
 - \mathcal S(t_{\mathrm{shift}}; v_{\mathrm{base}}, v_p)
$$

其中：

- 第一项是显式预处理已经走过的距离；
- 第二项是完整标准加速器的总位移；
- 第三项是因为“时间平移吸收”而需要扣掉的前置位移。

记起点侧与终点反演侧分别为：

$$
D_s(v_p),\qquad D_r(v_p)
$$

则完整轨迹满足两种情况。

### 6.1 存在匀速段

若把峰值速度直接取到上限 $v_m$ 后，仍有剩余路程：

$$
D_s(v_m)+D_r(v_m) < L
$$

则中间存在匀速段，其长度为：

$$
x_c = L-D_s(v_m)-D_r(v_m)
$$

对应匀速时长为：

$$
T_c=\frac{x_c}{v_m}
$$

这就是代码里先试探 `vm`，若 `x_const > 0` 就直接进入“有匀速段”分支的原因。

### 6.2 不存在匀速段

若在 $v_p=v_m$ 时两侧已经把路程几乎用满，说明轨迹只能是“加速后立即减速”。

这时要求解

$$
F(v_p)=D_s(v_p)+D_r(v_p)-L=0
$$

搜索区间为：

$$
v_p \in [v_{p,\min},\, v_m]
$$

其中

$$
v_{p,\min}=\max(v_{p,\min,s},\,v_{p,\min,r})
$$

在这一区间里，$F(v_p)$ 随 $v_p$ 单调增加，因此可以稳定地使用二分搜索。代码采用的容差为：

$$
\varepsilon = \texttt{S\_CURVE\_MAX\_BS\_ERROR}
$$

也就是 `EvaluateDistanceDelta()` 与后续 `while (r - l > ...)` 这段逻辑。

## 7. 总时长与阶段边界

若某一侧标准加速器总时长为 $T_{\mathrm{acc}}$，则考虑时间平移后的该侧有效时长为：

$$
T_s=t_{\mathrm{pre},s}+T_{\mathrm{acc},s}-t_{\mathrm{shift},s}
$$

$$
T_r=t_{\mathrm{pre},r}+T_{\mathrm{acc},r}-t_{\mathrm{shift},r}
$$

因此总时长为：

$$
T=
\begin{cases}
T_s+T_c+T_r, & \text{有匀速段} \\
T_s+T_r, & \text{无匀速段}
\end{cases}
$$

代码中的对应成员为：

- `t1_pre_`：起点显式预处理时长；
- `ts1_`：起点侧时间平移；
- `t3_pre_`：终点反演侧显式预处理时长；
- `ts3_`：终点反演侧时间平移；
- `t1_`：主加速段结束时刻；
- `t2_`：匀速段结束时刻；
- `total_time_`：整条轨迹总时长。

## 8. 轨迹采样公式

求解完成后，`CalcX()`、`CalcV()`、`CalcA()` 的实现本质上就是按时间区间做分段求值。

### 8.1 起点预处理段

当

$$
0\le t<t_{\mathrm{pre},s}
$$

时，轨迹直接使用常 `jerk` 多项式：

$$
\bar x(t)=\bar v_s t+\frac{1}{2}\bar a_s t^2+\frac{1}{6}j_m t^3
$$

$$
\bar v(t)=\bar v_s+\bar a_s t+\frac{1}{2}j_m t^2
$$

$$
\bar a(t)=\bar a_s+j_m t
$$

### 8.2 主加速段

当 $t$ 落在主加速段内时，令

$$
\hat t=t-t_{\mathrm{pre},s}+t_{\mathrm{shift},s}
$$

则：

$$
\bar x(t)
=x_{\mathrm{pre},s}
 + \mathcal S(\hat t; v_{\mathrm{base},s}, v_p)
 - \mathcal S(t_{\mathrm{shift},s}; v_{\mathrm{base},s}, v_p)
$$

$$
\bar v(t)=\mathcal V(\hat t; v_{\mathrm{base},s}, v_p)
$$

$$
\bar a(t)=\mathcal A(\hat t; v_{\mathrm{base},s}, v_p)
$$

这正对应代码里的：

- `process1_.getDistance(...) - xs1_`
- `process1_.getVelocity(...)`
- `process1_.getAcceleration(...)`

其中 `xs1_ = process1_.getDistance(ts1_)` 就是上式里被扣掉的那一段前置位移。

### 8.3 匀速段

若存在匀速段，则有

$$
\bar v(t)=v_p,\qquad \bar a(t)=0
$$

$$
\bar x(t)=D_s(v_p)+v_p(t-t_1)
$$

这对应 `has_const_ == true` 时的那一段实现。

### 8.4 末端反演段

当轨迹进入最后一段时，不再直接正向积分，而是令：

$$
\tau=T-t
$$

先在反演坐标中求

$$
X_r(\tau),\qquad V_r(\tau),\qquad A_r(\tau)
$$

再映射回正向时间：

$$
\bar x(t)=L-X_r(T-t)
$$

$$
\bar v(t)=V_r(T-t)
$$

$$
\bar a(t)=-A_r(T-t)
$$

在代码的原始坐标系里，这三式对应为：

$$
x(t)=x_e-d\,X_r(T-t)
$$

$$
v(t)=d\,V_r(T-t)
$$

$$
a(t)=-d\,A_r(T-t)
$$

如果 $\tau<t_{\mathrm{pre},r}$，反演段还处在显式预处理阶段，则有：

$$
X_r(\tau)=\bar v_e\tau-\frac{1}{2}\bar a_e\tau^2+\frac{1}{6}j_m\tau^3
$$

$$
V_r(\tau)=\bar v_e-\bar a_e\tau+\frac{1}{2}j_m\tau^2
$$

$$
A_r(\tau)=-\bar a_e+j_m\tau
$$

否则进入反演侧的标准单边加速器，并像起点侧一样扣除时间平移前的位移基准。

## 9. 可行性判据与失败条件

当前实现中，以下情况会直接返回 `success_ = false`：

### 9.1 零位移但边界状态不一致

若

$$
L \approx 0
$$

则只有在

$$
\bar v_s=\bar v_e,\qquad \bar a_s=\bar a_e
$$

时才可视为一条零时长轨迹；否则无解。

### 9.2 初末边界本身超限

若

$$
|\bar v_s|>v_m,\quad |\bar a_s|>a_m,\quad |\bar v_e|>v_m,\quad |\bar a_e|>a_m
$$

中的任意一项成立，则无解。

### 9.3 单侧规整后已无法满足速度上界

例如在 $a_0<0$ 的情形下，若

$$
|v_{\mathrm{base}}|>v_m
$$

说明即使只是把加速度抬回 0，速度也已经超限；因此该侧无解。

### 9.4 两侧显式预处理位移已经超过总路程

若

$$
L-x_{\mathrm{pre},s}-x_{\mathrm{pre},r}<-\varepsilon
$$

说明还没开始主过程，两端显式预处理就已经把路走“超了”，因此无解。

### 9.5 峰值速度区间为空

若

$$
v_m < \max(v_{p,\min,s},v_{p,\min,r})
$$

说明全局速度上限不足以同时满足起点侧和终点侧的最低峰值要求，因此无解。

### 9.6 二分结束后仍无法匹配位移

即便完成二分，如果最终仍有

$$
D_s(v_p)+D_r(v_p)-L>\varepsilon
$$

则说明当前约束下无法构造出满足位移闭合的轨迹，代码同样返回失败。

## 10. 数学符号与代码成员的对应关系

为了便于直接对照实现，可把主要符号与成员变量对应如下：

- $d$ ↔ `direction_`
- $L=|x_e-x_s|$ ↔ `len`
- $v_p$ ↔ `vp_`
- $t_{\mathrm{pre},s}$ ↔ `t1_pre_`
- $x_{\mathrm{pre},s}$ ↔ `start.x_pre`，以及绝对坐标中的 `x1_pre_`
- $t_{\mathrm{shift},s}$ ↔ `ts1_`
- $\mathcal S(t_{\mathrm{shift},s})$ ↔ `xs1_`
- $t_{\mathrm{pre},r}$ ↔ `t3_pre_`
- $x_{\mathrm{pre},r}$ ↔ `x3_pre_`
- $t_{\mathrm{shift},r}$ ↔ `ts3_`
- $\mathcal S(t_{\mathrm{shift},r})$ ↔ `xs3_`
- 主加速结束时刻 ↔ `t1_`
- 匀速结束时刻 ↔ `t2_`
- 总时长 $T$ ↔ `total_time_`

## 11. 总结

这套实现的核心并不是“硬凑一个长分段公式”，而是把复杂边界拆成三个简单问题：

1. 用 `SCurveAccel` 解决“从零加速度起步到零加速度收尾”的标准单边过程；
2. 用 `prepareSide` 把任意单侧边界规整成标准过程可以接受的形式；
3. 用时间反演把终点边界转成与起点完全同构的问题。

这样做的直接收益是：

- 数学结构清晰，推导和代码一一对应；
- 起点与终点处理逻辑统一，便于维护；
- 是否存在匀速段、峰值速度是多少，都可以转化为单变量 $v_p$ 的距离匹配问题；
- 最终采样阶段只需按时间落在哪个区间做分段求值即可。

从实现角度看，`s_curve.cpp` 本质上就是把上述数学构造翻译成了：

- 一个标准单边加速器；
- 一个单侧规整器；
- 一个峰值速度求解器；
- 一组正向/反向的分段采样函数。
