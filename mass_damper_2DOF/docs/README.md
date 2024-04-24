# Mass damper (2DOF)

## System equations

```math
\begin{cases}
m_1 \ddot{q}_1(t) + d_1 \dot{q}_1(t) + d_2 [\dot{q}_1(t) - \dot{q}_2(t)] = f_1(t) \\

m_2 \ddot{q}_2(t) + d_2 [\dot{q}_2(t) - \dot{q}_1(t)] = f_2(t)
\end{cases}
```

| Description | Symbol and unit |
|-|-|
| position | $`q_1, q_2 \mathrm{[m]}`$ |
| force | $`f_1, f_2 \mathrm{[N]}`$ |
| mass | $`m_1, m_2 \mathrm{[kg]}`$ |
| damping coefficient | $`d_1, d_2 \mathrm{[N \cdot s/m]}`$ |

## State space equation (plant_ode.m)

```math
\frac{d}{dt} \left[ \begin{array}{c}
q_1(t) \\ q_2(t) \\ \dot{q}_1(t) \\ \dot{q}_2(t)
\end{array} \right]
=
\left[ \begin{array}{c}
\dot{q}_1(t) \\
\dot{q}_2(t) \\
-\frac{d_1 + d_2}{m_1} \dot{q}_1(t) + \frac{d_2}{m_1} \dot{q}_2(t) + \frac{1}{m_1} f_1(t) \\
\frac{d_2}{m_2} \dot{q}_1(t) - \frac{d_2}{m_2} \dot{q}_2(t) + \frac{1}{m_2} f_2(t)
\end{array} \right]
=:
f(x(t), u(t))
```

## Linear state space equation (plant_sysc.m)

### Equilibrium point

The equilibrium point satisfies $`f(x_e, u_e) = 0`$ thus,

```math
\begin{cases}
\dot{q}_{1e} = 0 \\
\dot{q}_{2e} = 0 \\
f_{1e} = 0 \\
f_{2e} = 0
\end{cases}
```

where $`q_{1e}`$ and $`q_{2e}`$ is arbitrary.

### Linear state space equation

```math
\frac{d}{dt} \left[ \begin{array}{c}
q_1(t) \\ q_2(t) \\ \dot{q}_1(t) \\ \dot{q}_2(t)
\end{array} \right]
=
\left[ \begin{array}{cccc}
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1 \\
0 & 0 & -\frac{d_1 + d_2}{m_1} & \frac{d_2}{m_1} \\
0 & 0 & \frac{d_2}{m_2} & -\frac{d_2}{m_2}
\end{array} \right]

\left[ \begin{array}{c}
q_1(t) \\ q_2(t) \\ \dot{q}_1(t) \\ \dot{q}_2(t)
\end{array} \right]
+
\left[ \begin{array}{cc}
0 & 0 \\
0 & 0 \\
\frac{1}{m_1} & 0 \\
0 & \frac{1}{m_2}
\end{array} \right]

\left[ \begin{array}{c}
f_1(t) \\ f_2(t)
\end{array} \right]
```

## Simscape (plant_simscape.slx)

![simscape model](simscape.png)

## Simulation

### Parameters (plant_param.m)

| Parameter | Value |
|-|-|
| $`m_1 \mathrm{[kg]}`$ | $`1.0`$ |
| $`d_1 \mathrm{[N \cdot s/m]}`$ | $`1.0`$ |
| $`m_2 \mathrm{[kg]}`$ | $`1.0`$ |
| $`d_2 \mathrm{[N \cdot s/m]}`$ | $`1.0`$ |

### Initial state response (plot_initial.m)

![initial state response](initial.png)

where $`x(0) = [0, 0, 1, 0]^T`$, $`x_e = [0, 0, 0, 0]^T`$, $`u_e = [0, 0]^T`$.

### Impulse response (plot_impulse.m)

![impulse response](impulse.png)

where $`u = [\delta(t), 0]^T`$, $`x_e = [0, 0, 0, 0]^T`$, $`u_e = [0, 0]^T`$.

### Bode plot (plot_bode.m)

![bode plot from f1 to q1](bode1.png)

where $`G_1(s) = Q_1(s)/F_1(s)`$, $`x_e = [0, 0, 0, 0]^T`$, $`u_e = [0, 0]^T`$.

![bode plot from f1 to q2](bode2.png)

where $`G_2(s) = Q_2(s)/F_1(s)`$, $`x_e = [0, 0, 0, 0]^T`$, $`u_e = [0, 0]^T`$.
