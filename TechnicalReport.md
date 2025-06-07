

# Technical Report: Simulation of a Backward Kinematic Model of a 4-Wheeled Mecanum Drive Basis on a Circular Path

## 1. Introduction

This technical report describes the simulation of a 4-wheeled Mecanum robot performing a circular trajectory. The simulation was implemented in Python using Matplotlib to visualize the robot’s motion and individual wheel angular velocities. The report includes theoretical background, implementation details, and visualization strategies.

## 2. Theoretical Background

Mecanum wheels allow omnidirectional movement by combining the rotation of individually driven wheels mounted at 45-degree angles. When performing a circular path, the velocities of individual wheels can be determined from the desired robot translational and rotational velocities.

Given:

* $R$: Wheel radius
* $a, b$: Half distances of the robot's length and width respectively
* $v_x, v_y$: Translational velocities in the x and y directions
* $\omega_z$: Angular velocity about the z-axis

The angular velocity for each wheel is calculated using the inverse kinematics for a 4-wheel Mecanum platform:

$$
\begin{align*}
\omega_1 &= \frac{1}{R}(v_x - v_y - (a + b)\omega_z) \\
\omega_2 &= \frac{1}{R}(v_x + v_y + (a + b)\omega_z) \\
\omega_3 &= \frac{1}{R}(v_x + v_y - (a + b)\omega_z) \\
\omega_4 &= \frac{1}{R}(v_x - v_y + (a + b)\omega_z)
\end{align*}
$$

For circular motion of radius $r$, with constant speed:

$$
\omega_z = \frac{v}{r}
$$

where $v = \frac{2\pi r}{T}$, and $T = n \cdot dt$.

The velocities $v_x, v_y$ at any time $t$ are:

$$
\begin{align*}
v_x &= -r \cdot \omega_z \cdot \sin(\theta) \\
v_y &= r \cdot \omega_z \cdot \cos(\theta)
\end{align*}
$$


where $\theta = \omega_z \cdot t$.

These values are used to compute each wheel’s angular velocity over time.


## 3. Implementation

The simulation was implemented in Python using the following key components:

* **Numpy** for mathematical operations
* **Matplotlib** for visualization and animation
* **A fixed circular path was generated**
* **Robot body and wheels were visualized**
* **Angular velocities $\omega_i$ for each wheel were plotted over time**

### Python Code Snippet Highlights

* Calculated path points:

```python
x_path = radius * np.cos(2 * np.pi * theta_path)
y_path = radius * np.sin(2 * np.pi * theta_path)
```

* Wheel angular velocity computation loop:

```python
for i in range(n):
    omega[i, 0] = (vxi - vyi - (a + b) * wz) / R
    omega[i, 1] = (vxi + vyi + (a + b) * wz) / R
    omega[i, 2] = (vxi + vyi - (a + b) * wz) / R
    omega[i, 3] = (vxi - vyi + (a + b) * wz) / R
```
## 4. Visualization

The simulation window displays:

* A circular path with a visible dashed line path
* The robot's real-time location with orientation
* Four vectors representing wheel angular velocities over time

The Matplotlib figure is maximized on startup for better visibility.

>*Note: Current project may contain poorly rendered vector sizes and directions*

![Preview of the Simulation](Example.png "Preview of the Simulation")

## 5. Issues
Issues that are currently being addressed:
* Incorrect vector rendering
* Synchronization with plotting of wheel angular velocity graph
* Loop settings for repeated simulations

## 6. Conclusion

The simulation demonstrates the behavior of a Mecanum-wheeled robot following a circular path. Angular velocities align with expected values and wheel behavior corresponds with the theoretical kinematics model. Visual elements like wheel rotation direction, dashed trajectory, and time-based angular velocity help validate the simulation.

---

*Prepared using Python 3.12, Matplotlib 3.10, and Numpy 2.2*
