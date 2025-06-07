import argparse
import numpy as p
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrow
from matplotlib.animation import FuncAnimation
import sys 

def main(R, a, b, T, radius):

    # Parameters
    dt: float = 0.1 #Timestep duration [s]
    n = int(T/dt) #Number of timesteps [-]
    
    # --- Circle Path Parameters ---
    omega = 2 * p.pi / T
    omega_z = omega / radius

    t = p.linspace(0, T, n)
    theta_path = omega * t
    x_path = radius * p.cos(theta_path - (p.pi / 2))
    y_path = radius * p.sin(theta_path - (p.pi / 2))

    x_robot = radius * p.cos(2*p.pi*theta_path - (p.pi / 2))
    y_robot = radius * p.sin(2*p.pi*theta_path - (p.pi / 2))
    
    vx = -radius * omega_z * p.sin(theta_path)
    vy = radius * omega_z * p.cos(theta_path)

    # --- Wheel Angular Velocities ---
    omega = p.zeros((n, 4))
    for i in range(n):
        vxi, vyi = vx[i], vy[i]
        wz = omega_z
        omega[i, 0] = (vxi - vyi - (a + b) * wz) / R
        omega[i, 1] = (vxi + vyi + (a + b) * wz) / R
        omega[i, 2] = (vxi + vyi - (a + b) * wz) / R
        omega[i, 3] = (vxi - vyi + (a + b) * wz) / R

    # --- Figure and Subplots ---
    fig, (ax_left, ax_right) = plt.subplots(1, 2, figsize=(10, 10))
    fig.tight_layout(pad=5.0)

    # --- Left: Robot movement ---
    ax_left.set_aspect('equal')
    ax_left.set_xlim(-radius - 0.5, radius + 0.5)
    ax_left.set_ylim(-radius - 0.5, radius + 0.5)
    ax_left.set_title("Backward Kinematic Model of a 4-Wheel Mecanum Drive Base")

    # Path
    path_line, = ax_left.plot(x_robot, y_robot, color='gray', label='Path')
    path_line.set_dashes([10, 10])
    robot_marker, = ax_left.plot([], [], 'ro', markersize=4, label='Robot Center')
    robot_frame, = ax_left.plot([], [], 'k', lw=1, ls='solid')

    wheel_colors = ['red', 'green', 'blue', 'magenta']
    wheel_lines = [ax_left.plot([], [], color=wheel_colors[i], lw=5)[0] for i in range(4)]

    # Arrows for wheels
    wheel_arrows = [None for _ in range(4)]
    omega_z_arrow = None
    v_arrow = None  # Robot velocity vector (global motion)
    rot_arrows = [None for _ in range(4)]  # ω_z × rᵢ vectors for each wheel
    total_velocity_arrows = [None for _ in range(4)]


    ax_left.grid()
    ax_left.legend()

    # --- Right: Angular Velocities ---
    labels = [r'$\omega_1$', r'$\omega_2$', r'$\omega_3$', r'$\omega_4$']
    for i in range(4):
        ax_right.plot(t, omega[:, i], label=labels[i], color=wheel_colors[i], alpha=0.2)

    line_segments = [ax_right.plot([], [], color=wheel_colors[i])[0] for i in range(4)]
    scatter_points = [ax_right.plot([], [], 'o', color=wheel_colors[i])[0] for i in range(4)]

    ax_right.set_xlim(t[0], t[-1])
    ax_right.set_ylim(p.min(omega) * 1.1, p.max(omega) * 1.1)
    ax_right.set_xlabel("Time [s]")
    ax_right.set_ylabel("Angular Velocity [rad/s]")
    ax_right.set_title("Wheel Angular Velocities Over Time")
    ax_right.grid(True)
    ax_right.legend()
    time_line = ax_right.axvline(t[0], color='black', linestyle='--')

    # --- Helper: Robot Frame Box ---
    def get_square_frame(x, y, yaw=0):
        corners = p.array([
            [b, a], [b, -a], [-b, -a], [-b, a], [b, a]
        ])
        rot = p.array([[p.cos(yaw), -p.sin(yaw)],
                    [p.sin(yaw),  p.cos(yaw)]])
        rotated = corners @ rot.T
        return rotated[:, 0] + x, rotated[:, 1] + y

    # --- Animation Frame Update ---
    def update(frame):
        nonlocal omega_z_arrow, wheel_arrows, v_arrow, rot_arrows, total_velocity_arrows

        x, y = x_path[frame], y_path[frame]
        robot_marker.set_data([x], [y])
        rx, ry = get_square_frame(x, y)
        robot_frame.set_data(rx, ry)

        ang = p.pi / 2
        offsets = [[b, a], [b, -a], [-b, -a], [-b, a]]

        # Remove old arrows
        for i in range(4):
            if wheel_arrows[i] is not None:
                wheel_arrows[i].remove()
                wheel_arrows[i] = None

        for i in range(4):
            wx = x + offsets[i][0]
            wy = y + offsets[i][1]

            dx = 0.1 * p.cos(ang)
            dy = 0.1 * p.sin(ang)

            # Set wheel lines above the arrows
            wheel_lines[i].set_data([wx - dx, wx + dx], [wy - dy, wy + dy])
            wheel_lines[i].set_zorder(1)

            # Wheel arrow under the wheel
            scale = 0.05
            arrow_length = omega[frame, i] * scale
            ax_comp = arrow_length * p.cos(ang)
            ay_comp = arrow_length * p.sin(ang)

            arrow = FancyArrow(wx, wy, ax_comp, ay_comp,
                            width=0.01, head_width=0.05, head_length=0.05,
                            color='black', zorder=2)
            wheel_arrows[i] = ax_left.add_patch(arrow)
            
        # Remove old rotational velocity arrows
        for i in range(4):
            if rot_arrows[i] is not None:
                rot_arrows[i].remove()
                rot_arrows[i] = None

        for i in range(4):
            wx = x + offsets[i][0]
            wy = y + offsets[i][1]

            # r_i vector from CM to wheel
            rx = offsets[i][0]
            ry = offsets[i][1]

            # v_rot = ω_z × r_i = [-ω_z * ry, ω_z * rx]
            vrot_x = -omega_z * ry
            vrot_y = omega_z * rx

            # Scale for visibility
            scale = 1.0
            arrow = FancyArrow(wx, wy, vrot_x * scale, vrot_y * scale,
                            width=0.01, head_width=0.05, head_length=0.05,
                            color='black', zorder=2)
            rot_arrows[i] = ax_left.add_patch(arrow)

        for i in range(4):
            if total_velocity_arrows[i] is not None:
                total_velocity_arrows[i].remove()
                total_velocity_arrows[i] = None

        scale_vi = 1.0  # scale factor for visibility

        for i in range(4):
            rx_i, ry_i = offsets[i]
            wx = x + rx_i
            wy = y + ry_i

            # ω_z × rᵢ = [-ω_z * ry_i, ω_z * rx_i]
            vx_rot = -omega_z * ry_i
            vy_rot = omega_z * rx_i

            # vᵢ = v + ω_z × rᵢ
            vx_total = vx[frame] + vx_rot
            vy_total = vy[frame] + vy_rot

            total_velocity_arrows[i] = FancyArrow(wx, wy, vx_total * scale_vi, vy_total * scale_vi,
                                      width=0.005, head_width=0.03, head_length=0.03,
                                      color='lime', zorder=1,
                                      label=r'$\vec{v}_i$' if frame == 0 and i == 0 else None)

            ax_left.add_patch(total_velocity_arrows[i])

        scale_rot = 1.0  # Visual scale for rotational vectors

        for i in range(4):
            # Position relative to robot center
            rx_i, ry_i = offsets[i]
            wx = x + rx_i
            wy = y + ry_i

            # v_rot = ω_z × rᵢ = ω_z * [-yᵢ, xᵢ]
            vx_rot = -omega_z * ry_i
            vy_rot = omega_z * rx_i

            # Draw arrow
            rot_arrows[i] = FancyArrow(wx, wy, vx_rot * scale_rot, vy_rot * scale_rot,
                           width=0.005, head_width=0.03, head_length=0.03,
                           color='orange', zorder=1,
                           label=r'$\omega_z \times \vec{r}_i$' if frame == 0 and i == 0 else None)

            ax_left.add_patch(rot_arrows[i])

        # ω_z arrow (under everything else except path)
        # if omega_z_arrow is not None:
        #     omega_z_arrow.remove()
        # omega_mag = omega_z * 5.0
        # omega_z_arrow = FancyArrow(x, y, 0, omega_mag,
        #                         width=0.01, head_width=0.05, head_length=0.05,
        #                         color='orange', zorder=0)
        # ax_left.add_patch(omega_z_arrow)    

        # ω curves
        time_line.set_xdata([t[frame]])
        for i in range(4):
            line_segments[i].set_data(t[:frame+1], omega[:frame+1, i])
            scatter_points[i].set_data([t[frame]], [omega[frame, i]])
            
        # Remove old translational velocity arrow
        if v_arrow is not None:
            v_arrow.remove()
            v_arrow = None

        # Draw translational velocity vector from robot center
        v_x = vx[frame]
        v_y = vy[frame]
        scale_v = 1.0  # adjust for visibility
        v_arrow = FancyArrow(x, y, v_x * scale_v, v_y * scale_v,
                            width=0.01, head_width=0.05, head_length=0.05,
                            color='blue', zorder=2,
                            label=r'$\vec{v}$' if frame == 0 else None)
        ax_left.add_patch(v_arrow)


        if frame == 0:
            ax_left.legend(loc='upper left', fontsize='small')
            
        return [robot_marker, robot_frame, *wheel_lines, *line_segments,
                *scatter_points, time_line, v_arrow, *wheel_arrows, *rot_arrows, *total_velocity_arrows]


    # --- Run Animation ---
    ani = FuncAnimation(fig, update, frames=n, interval=10, blit=True)

    # --- Maximize Window ---
    mng = plt.get_current_fig_manager()
    try:
        mng.window.state('zoomed')
    except AttributeError:
        try:
            mng.window.showMaximized()
        except AttributeError:
            mng.full_screen_toggle()

    arrow_patch = FancyArrow(0, 0, 0.5, 0, width=0.01, color='black')
    legend_proxies = [
        plt.Line2D([0], [0], color='gray', linestyle='--', label='Path'),
        plt.Line2D([0], [0], color='black', lw=1, label='Robot Frame'),
        plt.Line2D([0], [0], marker='o', color='red', label='Robot Center'),
        plt.Line2D([0], [0], color='cyan', lw=2, label=r'$\omega_z \times r_i$'),
        plt.Line2D([0], [0], color='black', lw=2, label=r'Wheel Force $\omega_i$'),
        plt.Line2D([0], [0], color='yellow', lw=2, label=r'Translational $\vec{v}$'),
        plt.Line2D([0], [0], color='limegreen', lw=2, label=r'Total $\vec{v}_i$')

    ]
    ax_left.legend(handles=legend_proxies, loc='upper left')

    plt.show()


if __name__ == '__main__':
    
    # --- Parameters ---
    parser = argparse.ArgumentParser(description="4-Wheel Mecanum Robot Simulation")

    parser.add_argument('--R', type=float, default=0.05, help='Wheel radius [m]')
    parser.add_argument('--a', type=float, default=0.3, help='Half of robot length [m]')
    parser.add_argument('--b', type=float, default=0.3, help='Half of robot width [m]')
    parser.add_argument('--T', type=int, default=30, help='Time of one period [s]')
    parser.add_argument('--radius', type=float, default=2.0, help='Radius of circular path [m]')

    args = parser.parse_args()
    
    main(R=args.R, a=args.a, b=args.b, T=args.T, radius=args.radius)