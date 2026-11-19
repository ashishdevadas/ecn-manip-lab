# Modelling of Manipulators: ROS 2 lab (KUKA KR16, Turret, UR-10)

My solutions to the six exercises of the **Modelling of Manipulators (MANIP)** lab at École Centrale de Nantes (M1 CORO-IMARO, 2026), built on the [ecn_manip](https://github.com/oKermorgant/ecn_manip) ROS 2 simulator. For each exercise: the goal, the method with its key equations, and a video of the result.

| # | Exercise | What the robot does | Video |
|---|---|---|---|
| 1 | Direct geometric model | The model matches the simulator for any joint values | [▶ Ex. 1](VIDEO_LINK_EX1) |
| 2 | Jacobian | The tool moves in straight lines and rotates about its tip with the sliders | [▶ Ex. 2](VIDEO_LINK_EX2) |
| 3 | Inverse geometric model | The robot computes the joints that reach a given pose | [▶ Ex. 3](VIDEO_LINK_EX3) |
| 4 | Interpolated point-to-point | Smooth joint-space motion in minimum time | [▶ Ex. 4](VIDEO_LINK_EX4) |
| 5 | Straight line via inverse geometry | The tool follows an exact straight line in 1 s | [▶ Ex. 5](VIDEO_LINK_EX5) |
| 6 | Straight line via Jacobian | Velocity control, exponential convergence to the target | [▶ Ex. 6](VIDEO_LINK_EX6) |

**Comparison of Exercises 4, 5 and 6 on the same targets:** [▶ video](VIDEO_LINK_COMPARISON)

---

## The robots

| Robot | Joints | Model |
|---|---|---|
| Turret | RRP (3 DOF) | MDH table in [`models/turret.yml`](models/turret.yml) |
| KUKA KR16 | 6R, spherical wrist | MDH table in [`models/kr16.yml`](models/kr16.yml) |
| UR-10 | 6R, no spherical wrist | Inverse geometry provided by the lab |

Frames: **f** fixed base, **w** wrist, **e** end-effector. MDH notation: $(\alpha_i, a_i, \theta_i, r_i)$.

---

## Exercise 1: Direct geometric model

**Goal:** compute the pose of the wrist ${}^f\mathbf{M}_w(\mathbf{q})$ and the constant tool offset ${}^w\mathbf{M}_e$.

**Method:** product of the MDH transforms $\;{}^f\mathbf{M}_w = \prod_i \mathbf{R}_x(\alpha_i)\,\mathbf{T}_x(a_i)\,\mathbf{R}_z(\theta_i)\,\mathbf{T}_z(r_i)$, generated with `dh_code.py` from the tables in [`models/`](models/).

**Result:** in *Manual joint control*, `checkPose` shows the model pose equal to the simulator pose for any joint values.



https://github.com/user-attachments/assets/505f159f-b950-4e9c-87cb-41a685a61f60





## Exercise 2: Jacobian

**Goal:** drive the tool with a velocity (twist) given in the end-effector frame.

**Method:** express the twist in the fixed frame, then use the pseudo-inverse of the Jacobian:

$$\dot{\mathbf{q}} = \mathbf{J}^{+}(\mathbf{q}) \begin{bmatrix} {}^f\mathbf{R}_e & 0 \\ 0 & {}^f\mathbf{R}_e \end{bmatrix} {}^e\mathbf{v}$$

Linear rows of $\mathbf{J}$: derivative of the wrist position. Angular rows: the joint axes (rotation) or zero (prismatic joint).

**Result:** each translation slider moves the tool in a straight line, each rotation slider turns it about its tip.



https://github.com/user-attachments/assets/e72281ae-2816-4dc2-bc3b-630bcdf5ce85



## Exercise 3: Inverse geometric model

**Goal:** find all joint values $\mathbf{q}$ such that ${}^f\mathbf{M}_e(\mathbf{q}) = \mathbf{M}_d$.

**Method:**
- **Turret:** $q_1$ from $-t_x \sin q_1 + t_y \cos q_1 = 0$, then $(q_2, q_3)$ from $-q_3 \sin q_2 = \rho$ and $q_3 \cos q_2 = t_z - b$.
- **KR16 (spherical wrist):** the wrist position gives $q_1, q_2, q_3$; the orientation ${}^3\mathbf{R}_6 = {}^0\mathbf{R}_3^T\,\mathbf{R}_d$ gives $q_4, q_5, q_6$, with the wrist singularity ($\sin q_5 = 0$) handled separately.
- All candidates are checked against the pose and the joint limits; the one closest to the current configuration is used.

**Result:** in *Direct point-to-point*, the tool reaches every target pose.



https://github.com/user-attachments/assets/3979cfaa-a9a5-4fa6-8f64-3cb23f10b0ee





## Exercise 4: Interpolated point-to-point

**Goal:** move smoothly from $\mathbf{q}_0$ to $\mathbf{q}_f$ without exceeding the joint limits $\dot q_{max}$, $\ddot q_{max}$.

**Method:** 5th-order polynomial with zero velocity and acceleration at both ends:

$$\mathbf{q}(t) = \mathbf{q}_0 + p(x)(\mathbf{q}_f - \mathbf{q}_0), \qquad p(x) = 10x^3 - 15x^4 + 6x^5, \qquad x = \frac{t - t_0}{t_f}$$

Minimum time over all joints: $\; t_f = \max_i \left( \dfrac{15\,|D_i|}{8\,\dot q_{max,i}},\ \sqrt{\dfrac{10\,|D_i|}{\sqrt{3}\,\ddot q_{max,i}}} \right)$, with $D_i = q_{f,i} - q_{0,i}$.

**Result:** smooth motion, but the tool path is **curved** (only the joints are controlled).




https://github.com/user-attachments/assets/b886c9c1-67c6-492b-a4df-719751e297a1



## Exercise 5: Straight line via inverse geometry

**Goal:** move the tool along a straight line from $\mathbf{M}_0$ to $\mathbf{M}_d$ in $t_f = 1$ s.

**Method:** progress $\alpha = \dfrac{t - t_0}{t_f} \in [0, 1]$, then an intermediate pose on the line, solved by the inverse geometry at every cycle (100 Hz):

$$\mathbf{p}(\alpha) = (1-\alpha)\,\mathbf{p}_0 + \alpha\,\mathbf{p}_d, \qquad \mathbf{R}(\alpha) = \mathbf{R}_0\,\mathrm{Rot}(\mathbf{u}, \alpha\theta), \qquad \mathbf{q} = \mathrm{IGM}\big(\mathbf{M}_i(\alpha)\big)$$

where $\theta\mathbf{u}$ is the rotation $\mathbf{R}_0^T\mathbf{R}_d$.

**Result:** an exact straight line in 1 s. Limits: every pose on the line must be reachable, and a 3-DOF robot (Turret) can follow the position but not the orientation.




https://github.com/user-attachments/assets/0c1cd38d-4dbd-44af-9e10-18e5af8f6ffa



## Exercise 6: Straight line via Jacobian

**Goal:** reach $\mathbf{M}_d$ by sending joint velocities proportional to the pose error.

**Method:** pose error ${}^{e^*}\mathbf{M}_e = \mathbf{M}_d^{-1}\mathbf{M} \rightarrow (\mathbf{t}, \theta\mathbf{u})$, then

$$\mathbf{v} = -\lambda\,\mathbf{R}_d\,\mathbf{t}, \qquad \boldsymbol{\omega} = -\lambda\,\mathbf{R}\,\theta\mathbf{u}, \qquad \dot{\mathbf{q}} = \mathbf{J}^{+} \begin{bmatrix} \mathbf{v} \\ \boldsymbol{\omega} \end{bmatrix}$$

Since $\mathbf{R}_d\mathbf{t} = \mathbf{p} - \mathbf{p}_d$, the error obeys $\dot{\mathbf{e}} = -\lambda\mathbf{e}$, so $\mathbf{p}(t) - \mathbf{p}_d = (\mathbf{p}_0 - \mathbf{p}_d)\,e^{-\lambda t}$: a straight line with exponential convergence (5 % of the error left after $3/\lambda$ s).

**Result:** nearly straight path, fast then slow; a higher gain $\lambda$ converges faster. Limit: near a singularity $\mathbf{J}^{+}$ gives very large joint velocities.




https://github.com/user-attachments/assets/65223f9d-779a-4d6b-bd14-313c37a4ff67



---

## Comparison of Exercises 4, 5 and 6

| | Ex. 4 | Ex. 5 | Ex. 6 |
|---|---|---|---|
| Command | Joint positions | Joint positions | Joint velocities |
| Tool path | Curved | Straight | Nearly straight |
| Duration | Minimum time ($\dot q_{max}$, $\ddot q_{max}$) | Fixed, 1 s | Exponential ($\lambda$) |
| Needs | IGM once | IGM every cycle | Jacobian only |
| Weak point | No control of the path | Unreachable poses on the line | Singularities |

---

## How to run

```bash
# terminal 1: simulator (robot:=turret | kr16 | ur10)
ros2 launch ecn_manip simulation_launch.py robot:=kr16

# terminal 2: copy the files of src/ into your ecn_manip package, then build and run
cd ~/ros2 && colcon build --packages-select ecn_manip && source install/setup.bash
ros2 run ecn_manip control
```

Then choose the exercise with the buttons of the GUI: *Manual joint control* (1), *Manual Cartesian control* (2), *Direct point-to-point* (3), *Interpolated point-to-point* (4), *Straight line point-to-point* (5), *Velocity point-to-point* (6).

## Repository content

```
src/       control.cpp and the robot models (robot_turret.cpp, robot_kr16.cpp, robot_ur10.cpp)
models/    MDH tables for dh_code.py
media/     screenshots and GIFs
report/    lab report (PDF)
```

## Credits

- Simulator and lab framework: [ecn_manip](https://github.com/oKermorgant/ecn_manip) by Olivier Kermorgant (École Centrale de Nantes), MIT licence. The files in `src/` are the lab templates completed with my solutions.
- Course: Modelling of Manipulators, Prof. Andrea Cherubini, École Centrale de Nantes.
- Related project: [inspection-informed shared control on the KR16](https://github.com/ashishdevadas/kr16-shared-control).
