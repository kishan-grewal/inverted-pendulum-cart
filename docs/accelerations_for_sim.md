# Cart-Pendulum Dynamics Derivation

## Variables

**System parameters (constants):**
- $M$ - cart mass [kg]
- $m_{pend}$ - total pendulum mass (rod + tip) [kg]
- $l$ - distance from pivot to pendulum CoM [m]
- $I_{com}$ - moment of inertia of pendulum about its own CoM [kg·m²]
- $I_{pivot}$ - moment of inertia of pendulum about pivot [kg·m²]
- $g$ - gravitational acceleration [m/s²]
- $b_x$ - cart viscous friction coefficient [N·s/m]
- $b_\theta$ - pivot viscous friction coefficient [N·m·s/rad]

**State variables (time-varying):**
- $x$ - cart position [m]
- $\dot{x}$ - cart velocity [m/s]
- $\theta$ - pendulum angle from vertical [rad]
- $\dot{\theta}$ - pendulum angular velocity [rad/s]

**Input:**
- $F$ - horizontal force on cart [N]

**Derived quantities:**
- $M_t = M + m_{pend}$ - total translating mass
- $ml = m_{pend} \cdot l$ - mass-length product

**Sign conventions:**
- Positive $x$ = rightward
- Positive $\theta$ = tilted right from upright
- Positive $F$ = force pushing cart rightward
- $\theta = 0$ = pendulum balanced upright

---

## Assumptions

- Cart moves horizontally, position $x$
- Pendulum pivots freely at a point on the cart
- $\theta$ = angle from vertical, with $\theta = 0$ being **upright**
- Positive $\theta$ = pendulum tilted to the right
- Pendulum has combined mass $m_{pend}$, CoM at distance $l$ from pivot, moment of inertia $I_{pivot}$ about pivot
- Control input $F$ = horizontal force on cart (positive = rightward)

---

## Position of pendulum CoM

If $\theta = 0$ is upright, and positive $\theta$ tilts right:

$$x_p = x + l \sin\theta$$
$$y_p = l \cos\theta$$

---

## Velocities

$$\dot{x}_p = \dot{x} + l \dot{\theta} \cos\theta$$
$$\dot{y}_p = -l \dot{\theta} \sin\theta$$

---

## Kinetic energy

Cart:
$$T_{cart} = \frac{1}{2} M \dot{x}^2$$

Pendulum has translational KE of its CoM plus rotational KE about its CoM:
$$T_{pend} = \frac{1}{2} m_{pend} (\dot{x}_p^2 + \dot{y}_p^2) + \frac{1}{2} I_{com} \dot{\theta}^2$$

Expanding the velocity squared terms:
$$\dot{x}_p^2 + \dot{y}_p^2 = (\dot{x} + l\dot{\theta}\cos\theta)^2 + (-l\dot{\theta}\sin\theta)^2$$
$$= \dot{x}^2 + 2l\dot{x}\dot{\theta}\cos\theta + l^2\dot{\theta}^2\cos^2\theta + l^2\dot{\theta}^2\sin^2\theta$$
$$= \dot{x}^2 + 2l\dot{x}\dot{\theta}\cos\theta + l^2\dot{\theta}^2$$

So:
$$T_{pend} = \frac{1}{2} m_{pend} (\dot{x}^2 + 2l\dot{x}\dot{\theta}\cos\theta + l^2\dot{\theta}^2) + \frac{1}{2} I_{com} \dot{\theta}^2$$

Total kinetic energy:
$$T = \frac{1}{2} M \dot{x}^2 + \frac{1}{2} m_{pend} \dot{x}^2 + m_{pend} l \dot{x}\dot{\theta}\cos\theta + \frac{1}{2} m_{pend} l^2 \dot{\theta}^2 + \frac{1}{2} I_{com} \dot{\theta}^2$$

$$T = \frac{1}{2}(M + m_{pend})\dot{x}^2 + m_{pend} l \dot{x}\dot{\theta}\cos\theta + \frac{1}{2}(I_{com} + m_{pend} l^2)\dot{\theta}^2$$

Define $I_{pivot} = I_{com} + m_{pend} l^2$ (parallel axis theorem):
$$T = \frac{1}{2}(M + m_{pend})\dot{x}^2 + m_{pend} l \dot{x}\dot{\theta}\cos\theta + \frac{1}{2} I_{pivot} \dot{\theta}^2$$

---

## Potential energy

Taking $y = 0$ at the pivot (on the cart), pendulum CoM is at height $y_p = l\cos\theta$:
$$V = m_{pend} g l \cos\theta$$

---

## Lagrangian

$$\mathcal{L} = T - V = \frac{1}{2}(M + m_{pend})\dot{x}^2 + m_{pend} l \dot{x}\dot{\theta}\cos\theta + \frac{1}{2} I_{pivot} \dot{\theta}^2 - m_{pend} g l \cos\theta$$

---

## Euler-Lagrange for $x$

$$\frac{\partial \mathcal{L}}{\partial \dot{x}} = (M + m_{pend})\dot{x} + m_{pend} l \dot{\theta}\cos\theta$$

$$\frac{d}{dt}\frac{\partial \mathcal{L}}{\partial \dot{x}} = (M + m_{pend})\ddot{x} + m_{pend} l \ddot{\theta}\cos\theta - m_{pend} l \dot{\theta}^2 \sin\theta$$

$$\frac{\partial \mathcal{L}}{\partial x} = 0$$

Equation (with external force $F$ and damping):
$$(M + m_{pend})\ddot{x} + m_{pend} l \ddot{\theta}\cos\theta - m_{pend} l \dot{\theta}^2 \sin\theta = F - b_x \dot{x}$$

---

## Euler-Lagrange for $\theta$

$$\frac{\partial \mathcal{L}}{\partial \dot{\theta}} = m_{pend} l \dot{x}\cos\theta + I_{pivot}\dot{\theta}$$

$$\frac{d}{dt}\frac{\partial \mathcal{L}}{\partial \dot{\theta}} = m_{pend} l \ddot{x}\cos\theta - m_{pend} l \dot{x}\dot{\theta}\sin\theta + I_{pivot}\ddot{\theta}$$

$$\frac{\partial \mathcal{L}}{\partial \theta} = -m_{pend} l \dot{x}\dot{\theta}\sin\theta + m_{pend} g l \sin\theta$$

Equation (with damping):
$$m_{pend} l \ddot{x}\cos\theta - m_{pend} l \dot{x}\dot{\theta}\sin\theta + I_{pivot}\ddot{\theta} - (-m_{pend} l \dot{x}\dot{\theta}\sin\theta + m_{pend} g l \sin\theta) = -b_\theta \dot{\theta}$$

Simplifying (the $m_{pend} l \dot{x}\dot{\theta}\sin\theta$ terms cancel):
$$m_{pend} l \ddot{x}\cos\theta + I_{pivot}\ddot{\theta} - m_{pend} g l \sin\theta = -b_\theta \dot{\theta}$$

---

## Final coupled equations

$$\boxed{(M + m_{pend})\ddot{x} + m_{pend} l \cos\theta \cdot \ddot{\theta} = F - b_x \dot{x} + m_{pend} l \dot{\theta}^2 \sin\theta}$$

$$\boxed{m_{pend} l \cos\theta \cdot \ddot{x} + I_{pivot} \ddot{\theta} = m_{pend} g l \sin\theta - b_\theta \dot{\theta}}$$

---

## Matrix form

$$\begin{bmatrix} M + m_{pend} & m_{pend} l \cos\theta \\ m_{pend} l \cos\theta & I_{pivot} \end{bmatrix} \begin{bmatrix} \ddot{x} \\ \ddot{\theta} \end{bmatrix} = \begin{bmatrix} F - b_x \dot{x} + m_{pend} l \dot{\theta}^2 \sin\theta \\ m_{pend} g l \sin\theta - b_\theta \dot{\theta} \end{bmatrix}$$

---

## Explicit solution for accelerations

Let:
- $M_t = M + m_{pend}$
- $c = \cos\theta$
- $s = \sin\theta$
- $ml = m_{pend} \cdot l$

Determinant:
$$D = M_t \cdot I_{pivot} - (ml \cdot c)^2$$

Right-hand side:
$$f_1 = F - b_x \dot{x} + ml \cdot \dot{\theta}^2 s$$
$$f_2 = ml \cdot g \cdot s - b_\theta \dot{\theta}$$

Inverse of 2×2 matrix $\begin{bmatrix} a & b \\ b & d \end{bmatrix}^{-1} = \frac{1}{ad - b^2}\begin{bmatrix} d & -b \\ -b & a \end{bmatrix}$

**Explicit accelerations:**

$$\ddot{x} = \frac{I_{pivot} \cdot f_1 - ml \cdot c \cdot f_2}{D}$$

$$\ddot{\theta} = \frac{M_t \cdot f_2 - ml \cdot c \cdot f_1}{D}$$

---

## State-space form for simulation

State vector: $\mathbf{x} = \begin{bmatrix} x \\ \dot{x} \\ \theta \\ \dot{\theta} \end{bmatrix}$

State derivatives: $\dot{\mathbf{x}} = \begin{bmatrix} \dot{x} \\ \ddot{x} \\ \dot{\theta} \\ \ddot{\theta} \end{bmatrix}$

where $\ddot{x}$ and $\ddot{\theta}$ are computed from the explicit formulas above.