import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrow
from matplotlib.animation import FuncAnimation
from matplotlib.artist import Artist

import sys

def compute_robot_path(path_type, radius, T, n, scale=1.0):
    t = np.linspace(0, T, n)
    
    if path_type == 'circle':    
        theta_path = 2 * np.pi * t / T
        x_path = radius * np.cos(theta_path - (np.pi / 2))
        y_path = radius * np.sin(theta_path - (np.pi / 2))

    elif path_type == 'square':
        edge_time = T / 4
        x_path = np.zeros_like(t)
        y_path = np.zeros_like(t)
        theta_path = np.zeros_like(t)
        
        for i, time in enumerate(t):
            phase = (time % T) / edge_time
            if phase < 1:
                x_path[i] = -radius + 2 * radius * phase
                y_path[i] = -radius
                theta_path[i] = 0
            elif phase < 2:
                x_path[i] = radius
                y_path[i] = -radius + 2 * radius * (phase - 1)
                theta_path[i] = np.pi / 2
            elif phase < 3:
                x_path[i] = radius - 2 * radius * (phase - 2)
                y_path[i] = radius
                theta_path[i] = np.pi
            else:
                x_path[i] = -radius
                y_path[i] = radius - 2 * radius * (phase - 3)
                theta_path[i] = -np.pi / 2

    elif path_type == 'triangle':
        edge_time = T / 3
        x_path = np.zeros_like(t)
        y_path = np.zeros_like(t)
        theta_path = np.zeros_like(t)

        angles = np.deg2rad([90, 210, 330])
        vertices = [(radius * np.cos(a), radius * np.sin(a)) for a in angles]

        for i, time in enumerate(t):
            phase = (time % T) / edge_time
            edge = int(phase) % 3
            local_phase = phase - edge

            x0, y0 = vertices[edge]
            x1, y1 = vertices[(edge + 1) % 3]

            x_path[i] = x0 + (x1 - x0) * local_phase
            y_path[i] = y0 + (y1 - y0) * local_phase
            theta_path[i] = np.arctan2(y1 - y0, x1 - x0)
    elif path_type == 'line_horizon':
        # Move from x = -radius to x = +radius, y = 0
        x_path = np.linspace(-radius, radius, n)
        y_path = np.zeros_like(x_path)
        theta_path = np.zeros_like(x_path)  # facing right (0 rad)
    elif path_type == 'line_vert':
        # Move from y = -radius to y = +radius, x = 0
        y_path = np.linspace(-radius, radius, n)
        x_path = np.zeros_like(y_path)
        theta_path = np.zeros_like(y_path)  # facing right (0 rad)

    else:
        raise ValueError("Unsupported path_type. Use 'circle', 'square', or 'triangle'.")

    # Apply global scaling
    x_path *= scale
    y_path *= scale

    return t, theta_path, x_path, y_path

def compute_velocities(radius, omega_z, theta_path, n):
    vx = -radius * omega_z * np.sin(theta_path)
    vy =  radius * omega_z * np.cos(theta_path)
    return vx, vy

def compute_velocity_vectors(x_path, y_path, dt):
    vx = np.gradient(x_path, dt)
    vy = np.gradient(y_path, dt)
    return vx, vy

def compute_omega_data(vx, vy, a, b, omega_z, R, n):
    omega_data = np.zeros((n, 4))
    for i in range(n):
        vxi, vyi = vx[i], vy[i]
        wz = omega_z
        omega_data[i, 0] = ( vxi - vyi + (-a - b) * wz) / R
        omega_data[i, 1] = (vxi + vyi + (a + b) * wz) / R
        omega_data[i, 2] = (vxi + vyi + (-a - b) * wz) / R
        omega_data[i, 3] = (vxi - vyi + (a + b) * wz) / R
    return omega_data

def compute_vi_data(vx, vy, a, b, omega_z, n):
    vi_data = np.zeros((n, 4))
    for i in range(n):
        vxi, vyi = vx[i], vy[i]
        wz = omega_z
        vi_data[i, 0] = vxi - vyi + (-a - b) * wz
        vi_data[i, 1] = vxi + vyi + (a + b) * wz
        vi_data[i, 2] = vxi + vyi + (-a - b) * wz
        vi_data[i, 3] = vxi - vyi + (a + b) * wz
    return vi_data

def compute_vri_data(vx, vy, omega_z, a, b, n):
    gamma = [np.pi/4, -np.pi/4, np.pi/4, -np.pi/4]
    offsets = [[b, a], [b, -a], [-b, -a], [-b, a]]
    vri_data = np.zeros((n, 4))
    for i in range(n):
        for w in range(4):
            rix, riy = offsets[w]
            v_rot_x = -omega_z * riy
            vix = vx[i] + v_rot_x

            cos_gamma = np.cos(gamma[w])
            cos_gamma = cos_gamma if abs(cos_gamma) >= 1e-6 else 1e-6
            vri_data[i, w] = (vx[i] - vix) / cos_gamma
    return vri_data

def setup_figure(radius, t, omega_data, vi_data, vri_data, scale):
    fig = plt.figure(figsize=(15, 10))

    ax_left = fig.add_subplot(1, 2, 1)
    ax_right_top = fig.add_subplot(2, 2, 2)
    ax_right_bot = fig.add_subplot(2, 2, 4)

    fig.tight_layout(pad=5.0)

    # Left plot setup
    ax_left.set_aspect('equal')
    ax_left.set_xlim((-radius - 0.5)*scale, (radius + 0.5)*scale)
    ax_left.set_ylim((-radius - 0.5)*scale, (radius + 0.5)*scale)
    ax_left.set_title("Backward Kinematic Model of a 4-Wheel Mecanum Drive Base")
    ax_left.grid()

    # Omega plot (top right)
    wheel_colors = ['red', 'green', 'blue', 'magenta']
    labels = [r'$\omega_1$', r'$\omega_2$', r'$\omega_3$', r'$\omega_4$']
    for i in range(4):
        ax_right_top.plot(t, omega_data[:, i], label=labels[i], color=wheel_colors[i], alpha=0.2)
    
    ax_right_top.set_xlim(t[0], t[-1])
    ax_right_top.set_ylim(np.min(omega_data) * 1.1, np.max(omega_data) * 1.1)
    ax_right_top.set_xlabel("Time [s]")
    ax_right_top.set_ylabel(r"Angular Velocity [$rad\cdot s^{-1}$]")
    ax_right_top.set_title("Wheel Angular Velocities Over Time")
    ax_right_top.grid(True)
    ax_right_top.legend(loc='best', fontsize='small')

    # vri plot (bottom right)
    vi_colors = ['darkcyan', 'purple', 'gold', 'forestgreen']
    vi_labels = [r'$v_{1}$', r'$v_{2}$', r'$v_{3}$', r'$v_{4}$']
    vri_colors = ['cyan', 'magenta', 'orange', 'lime']
    vri_labels = [r'$v_{r1}$', r'$v_{r2}$', r'$v_{r3}$', r'$v_{r4}$']
    for i in range(4):
        ax_right_bot.plot(t, vri_data[:, i], label=vri_labels[i], color=vri_colors[i], alpha=0.2)
    for i in range(4):
        ax_right_bot.plot(t, vi_data[:, i], label=vi_labels[i], color=vi_colors[i], alpha=0.2)

    ax_right_bot.set_xlim(t[0], t[-1])
    ax_right_bot.set_ylim(np.min(vi_data) * 1.1, np.max(vi_data) * 1.1)
    ax_right_bot.set_xlabel("Time [s]")
    ax_right_bot.set_ylabel(r" Velocity [$m \cdot s^{-1}$]")
    ax_right_bot.set_title("Calculated $v_{ri}$ and $v_{i}$ Over Time")
    ax_right_bot.grid(True)
    ax_right_bot.legend(fontsize='small')

    return fig, ax_left, ax_right_top, ax_right_bot, wheel_colors, vri_colors, vi_colors

def add_static_arrows(ax):
    cords = [
        {'dx': 1, 'dy': 0, 'color': 'limegreen', 'label': 'World X axis'},
        {'dx': 0, 'dy': 1, 'color': 'red', 'label': 'World Y axis'}
    ]
    for cords_params in cords:
        arrows = FancyArrow(0, 0, cords_params['dx'], cords_params['dy'],
                           width=0.001, head_width=0.05, head_length=0.05,
                           color=cords_params['color'], zorder=2, label=cords_params['label'])
        ax.add_patch(arrows)

def get_square_frame(x, y, a, b, yaw=0):
    corners = np.array([[b, a], [b, -a], [-b, -a], [-b, a], [b, a]])
    rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw),  np.cos(yaw)]])
    rotated = corners @ rot.T
    return rotated[:, 0] + x, rotated[:, 1] + y

def run_animation(R, a, b, T, radius, pathtype, scale):
    dt = 0.1
    n = int(T / dt)
    omega = 2 * np.pi / T
    omega_z = omega / radius

    t, theta_path, x_path, y_path = compute_robot_path(pathtype, radius, T, n, scale)
    # vx, vy = compute_velocities(radius, omega_z, theta_path, n)
    vx, vy = compute_velocity_vectors(x_path, y_path, dt)
    omega_data = compute_omega_data(vx, vy, a, b, omega_z, R, n)
    vi_data = compute_vi_data(vx, vy, a, b, omega_z, n)
    vri_data = compute_vri_data(vx, vy, omega_z, a, b, n)
    gamma = [np.pi/4, -np.pi/4, np.pi/4, -np.pi/4]

    fig, ax_left, ax_right_top, ax_right_bot, wheel_colors, vri_colors, vi_colors = setup_figure(radius, t, omega_data, vi_data, vri_data, scale)
    add_static_arrows(ax_left)

    path_line, = ax_left.plot(x_path, y_path, '-' ,color='gray', label='Path')
    path_line.set_dashes([10, 10])
    robot_marker, = ax_left.plot([], [], 'ro', markersize=4, label='Robot Center')
    robot_frame, = ax_left.plot([], [], 'k', lw=1, ls='solid')

    wheel_lines = [ax_left.plot([], [], color=wheel_colors[i], lw=5)[0] for i in range(4)]
    wheel_arrows = [None] * 4
    rot_arrows = [None] * 4
    # total_velocity_arrows = [None] * 4
    omega_z_arrow = None
    v_arrow = None
    vxy_arrows = [None] * 2
    
    line_segments = [ax_right_top.plot([], [], color=wheel_colors[i])[0] for i in range(4)]
    scatter_points = [ax_right_top.plot([], [], 'o', color=wheel_colors[i])[0] for i in range(4)]

    vri_lines = [ax_right_bot.plot([], [], color=vri_colors[i])[0] for i in range(4)]
    vri_scatter = [ax_right_bot.plot([], [], 'o', color=vri_colors[i])[0] for i in range(4)]
    
    vi_lines = [ax_right_bot.plot([], [], color=vi_colors[i])[0] for i in range(4)]
    vi_scatter = [ax_right_bot.plot([], [], 'o', color=vi_colors[i])[0] for i in range(4)]

    time_line = ax_right_top.axvline(t[0], color='black', linestyle='--')

    offsets = [[b, a], [b, -a], [-b, -a], [-b, a]]
    ang = np.pi / 2
    scale = 0.05

    def update(frame):
        print(f"Frame {frame} update start")
        nonlocal omega_z_arrow, wheel_arrows, rot_arrows, v_arrow, vxy_arrows

        x, y = x_path[frame], y_path[frame]
        robot_marker.set_data([x], [y])
        rx, ry = get_square_frame(x, y, a, b)
        robot_frame.set_data(rx, ry)

        # Remove old arrows
        for i in range(4):
            if wheel_arrows[i]: 
                wheel_arrows[i].remove()
                wheel_arrows[i] = None
            if rot_arrows[i]: 
                rot_arrows[i].remove()
                rot_arrows[i] = None
            # if total_velocity_arrows[i]: 
            #     total_velocity_arrows[i].remove()
            #     total_velocity_arrows[i] = None
        for i in range(2):
            if vxy_arrows[i]:
                vxy_arrows[i].remove()
                vxy_arrows[i] = None
        if v_arrow: 
            v_arrow.remove()
            v_arrow = None
        if omega_z_arrow: 
            omega_z_arrow.remove()
            omega_z_arrow = None
       

        # Draw wheels and omega arrows
        for i in range(4):
            wx = x + offsets[i][0]
            wy = y + offsets[i][1]

            dx = 0.1 * np.cos(ang)
            dy = 0.1 * np.sin(ang)

            wheel_lines[i].set_data([wx - dx, wx + dx], [wy - dy, wy + dy])
            wheel_lines[i].set_zorder(1)

            arrow_length = omega_data[frame, i] * scale
            ax_comp = arrow_length * np.cos(ang)
            ay_comp = arrow_length * np.sin(ang)

            wheel_arrows[i] = ax_left.add_patch(FancyArrow(wx, wy, ax_comp, ay_comp,
                width=0.01, head_width=0.05, head_length=0.05, color='black', zorder=2))
            
        # rotational velocity arrows
        # rotational velocity arrows - updated to match vri_data values with roller directions
        scale_vri = 2.0  # adjust scale for visibility

        for i in range(4):
            wx, wy = x + offsets[i][0], y + offsets[i][1]
            # Direction vector from gamma (roller angle)
            dir_x = np.cos(gamma[i])
            dir_y = np.sin(gamma[i])
            length = vri_data[frame, i] * scale_vri

            rot_arrows[i] = ax_left.add_patch(FancyArrow(
                wx, wy,
                length * dir_x,
                length * dir_y,
                width=0.005,
                head_width=0.03,
                head_length=0.03,
                color='cyan',
                zorder=2,
                label=r'$\vec{v_{ri}}$' if frame == 0 and i == 0 else None
            ))


        # translational velocity vector
        v_arrow = FancyArrow(x, y, vx[frame], vy[frame],
                             width=0.01, head_width=0.05, head_length=0.05,
                             color='blue', zorder=2,
                             label=r'$\vec{v}$' if frame == 0 else None)
        ax_left.add_patch(v_arrow)

        vxy_params = [{'color': 'red', 'dx': vx[frame], 'dy': 0},
                      {'color': 'green', 'dx': 0, 'dy': vy[frame]}]
        for i, vxy_param in enumerate(vxy_params):
            if vxy_arrows[i]:
                vxy_arrows[i].remove()
            vxy_arrows[i] = FancyArrow(x, y, vxy_param['dx'], vxy_param['dy'],
                                    width=0.001, head_width=0.05, head_length=0.05,
                                    color=vxy_param['color'], zorder=2)
            ax_left.add_patch(vxy_arrows[i])
            
        # total velocity vectors v_i
        # for i in range(4):
        #     wx, wy = x + offsets[i][0], y + offsets[i][1]
        #     rix, riy = offsets[i]
        #     v_rot_x = -omega_z * riy
        #     v_rot_y = omega_z * rix
        #     vix = vx[frame] + v_rot_x
        #     viy = vy[frame] + v_rot_y
        #     total_velocity_arrows[i] = ax_left.add_patch(FancyArrow(wx, wy, vix, viy,
        #         width=0.005, head_width=0.03, head_length=0.03,
        #         color='limegreen', zorder=2,
        #         label=r'$\vec{v}_i$' if frame == 0 and i == 0 else None))

        # Update omega plots
        time_line.set_xdata([t[frame]])
        for i in range(4):
            line_segments[i].set_data(t[:frame+1], omega_data[:frame+1, i])
            scatter_points[i].set_data([t[frame]], [omega_data[frame, i]])

        # Update vri, vi plots
        for i in range(4):
            vri_lines[i].set_data(t[:frame+1], vri_data[:frame+1, i])
            vri_scatter[i].set_data([t[frame]], [vri_data[frame, i]])
            vi_lines[i].set_data(t[:frame+1], vi_data[:frame+1, i])
            vi_scatter[i].set_data([t[frame]], [vi_data[frame, i]])

        if frame == 0:
            ax_left.legend(loc='best', fontsize='small')
            
        print("Update finished, checking artists...")

        items = [robot_marker, robot_frame, *wheel_lines, *line_segments,
                *scatter_points, time_line, v_arrow,
                *wheel_arrows, *rot_arrows,
                *vri_lines, *vri_scatter, *vi_lines, *vi_scatter, *vxy_arrows]

        for i, item in enumerate(items):
            if item is None:
                print(f"❌ Item {items[i]} is None")
            elif not isinstance(item, Artist):
                print(f"❌ Item {i} is not Artist: {type(item)}")
            else:
                print(f"✅ Item {i}: OK")

        return items

        # return [robot_marker, robot_frame, *wheel_lines, *line_segments,
        #         *scatter_points, time_line, v_arrow,
        #         *wheel_arrows, *rot_arrows,
        #         *vri_lines, *vri_scatter, *vi_lines, *vi_scatter, *vxy_arrows]

    ani = FuncAnimation(fig, update, frames=n, interval=10, blit=True)

    mng = plt.get_current_fig_manager()
    try:
        mng.window.state('zoomed')
    except AttributeError:
        try:
            mng.window.showMaximized()
        except AttributeError:
            mng.full_screen_toggle()

    plt.show()

def main(pathtype, R, a, b, T, radius, scale):
    run_animation(R, a, b, T, radius, pathtype, scale)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="4-Wheel Mecanum Robot Simulation")
    parser.add_argument('--R', type=float, default=0.05, help='Wheel radius [m]')
    parser.add_argument('--a', type=float, default=0.3, help='Half of robot length [m]')
    parser.add_argument('--b', type=float, default=0.3, help='Half of robot width [m]')
    parser.add_argument('--T', type=int, default=30, help='Time of one period [s]')
    parser.add_argument('--radius', type=float, default=2.0, help='Radius of circular path [m]')
    parser.add_argument('--pathtype', type=str, default='circle', help='Shape of the path')
    parser.add_argument('--scale', type=int, default=1, help='Scale of the path')
    
    args = parser.parse_args()
    
    main(R=args.R, a=args.a, b=args.b, T=args.T, radius=args.radius, pathtype=args.pathtype, scale=args.scale)
