import matplotlib
import matplotlib.pyplot as plt
import numpy as np

matplotlib.rcParams['lines.linewidth'] = 0.9


def plot(time, dtime, states: dict, w: float, h: float, colorset: int, title: str):
    dpi = 90
    fig, (
        #
        (ax_ref_ori,      ax_ref_pos),
        (ax_ori,          ax_pos),
        (ax_ang_vel,      ax_lin_vel),
        (ax_torque,       ax_thrust),
        (ax_torque_ctrl,  ax_thrust_ctrl)
        #
    ) = plt.subplots(nrows=5, ncols=2, figsize=(w/dpi, h/dpi), dpi=dpi)

    colors = [
        {'x': 'red', 'y': 'green', 'z': 'blue'},
        {'x': 'tomato', 'y': 'lightgreen', 'z': 'steelblue'},
    ]

    c = colors[colorset]

    # TODO: I don't know if the order of x, y, z for Euler angles is correct.

    if title:
        fig.suptitle(title)

    # ROW 0
    ax_ref_ori.plot(
        dtime, states['reference_orientation']['x'], color=c['x'], label='x')
    ax_ref_ori.plot(
        dtime, states['reference_orientation']['y'], color=c['y'], label='y')
    ax_ref_ori.plot(
        dtime, states['reference_orientation']['z'], color=c['z'], label='z')
    ax_ref_ori.set_title("Reference orientation")
    ax_ref_ori.set_ylabel("Euler Angles [$\\mathrm{rad}$]")
    ax_ref_ori.set_xlim(time[0], time[-1])
    ax_ref_ori.legend()
    #
    #
    ax_ref_pos.plot(dtime, states['reference_position']['x'], color=c['x'])
    ax_ref_pos.plot(dtime, states['reference_position']['y'], color=c['y'])
    ax_ref_pos.plot(dtime, states['reference_position']['z'], color=c['z'])
    ax_ref_pos.set_title("Reference position")
    ax_ref_pos.set_ylabel("Position [$m$]")
    ax_ref_pos.set_xlim(time[0], time[-1])
    #
    #
    # ROW 1
    ax_ori.plot(time, states['orientation']['x'], color=c['x'])
    ax_ori.plot(time, states['orientation']['y'], color=c['y'])
    ax_ori.plot(time, states['orientation']['z'], color=c['z'])
    ax_ori.set_title("Orientation")
    ax_ori.set_ylabel("Euler Angles [$\\mathrm{rad}$]")
    ax_ori.set_xlim(time[0], time[-1])
    #
    #
    ax_pos.plot(time, states['position']['x'], color=c['x'])
    ax_pos.plot(time, states['position']['y'], color=c['y'])
    ax_pos.plot(time, states['position']['z'], color=c['z'])
    ax_pos.set_title("Position")
    ax_pos.set_ylabel("Position [$m$]")
    ax_pos.set_xlim(time[0], time[-1])
    #
    #
    # ROW 2
    ax_ang_vel.plot(time, states['angular_velocity']['x'], color=c['x'])
    ax_ang_vel.plot(time, states['angular_velocity']['y'], color=c['y'])
    ax_ang_vel.plot(time, states['angular_velocity']['z'], color=c['z'])
    ax_ang_vel.set_title("Angular velocity")
    ax_ang_vel.set_ylabel("Angular velocity [$\\mathrm{rad}/s$]")
    ax_ang_vel.set_xlim(time[0], time[-1])
    #
    #
    ax_lin_vel.plot(time, states['linear_velocity']['x'], color=c['x'])
    ax_lin_vel.plot(time, states['linear_velocity']['y'], color=c['y'])
    ax_lin_vel.plot(time, states['linear_velocity']['z'], color=c['z'])
    ax_lin_vel.set_title("Velocity")
    ax_lin_vel.set_ylabel("Velocity [$m/s$]")
    ax_lin_vel.set_xlim(time[0], time[-1])
    #
    #
    # ROW 3
    ax_torque.plot(time, states['torque_motor_velocity']['x'], color=c['x'])
    ax_torque.plot(time, states['torque_motor_velocity']['y'], color=c['y'])
    ax_torque.plot(time, states['torque_motor_velocity']['z'], color=c['z'])
    ax_torque.set_title("Torque motor velocity")
    ax_torque.set_ylabel("Angular velocity [$?$]")
    ax_torque.set_xlim(time[0], time[-1])
    #
    #
    ax_thrust.plot(time, states['thrust_motor_velocity']['z'], color=c['z'])
    ax_thrust.set_title("Thrust motor velocity")
    ax_thrust.set_ylabel("Angular velocity [$?$]")
    ax_thrust.set_xlim(time[0], time[-1])
    #
    #
    # ROW 4
    ax_torque_ctrl.plot(dtime, states['torque_control']['x'], color=c['x'])
    ax_torque_ctrl.plot(dtime, states['torque_control']['y'], color=c['y'])
    ax_torque_ctrl.plot(dtime, states['torque_control']['z'], color=c['z'])
    ax_torque_ctrl.set_title("Torque motor control")
    ax_torque_ctrl.set_ylabel("Control signal [-]")
    ax_torque_ctrl.set_xlabel("Time [$s$]")
    ax_torque_ctrl.set_xlim(time[0], time[-1])
    #
    #
    ax_thrust_ctrl.plot(dtime, states['thrust_control']['z'], color=c['z'])
    ax_thrust_ctrl.set_title("Thrust motor control")
    ax_thrust_ctrl.set_ylabel("Control signal [-]")
    ax_thrust_ctrl.set_xlabel("Time [$s$]")
    ax_thrust_ctrl.set_xlim(time[0], time[-1])

    fig.tight_layout()
    return fig


def show(fig: matplotlib.figure.Figure):
    plt.show(fig)


def save(fig: matplotlib.figure.Figure, filename: str):
    fig.savefig(filename)


def plot_step_analyzer(axes: matplotlib.axes.Axes, result: dict, title: str, legends: list, colorset: int):
    colors = [
        ('red', 'green', 'blue'), # TODO: add more colors
        ('tomato', 'lightgreen', 'steelblue'),
    ]
    c = colors[colorset]

    n = len(result['states'])

    for i in range(n):
        if legends:
            axes.plot(result['times'], result['states']
                      [i], color=c[i], label=legends[i])
        else:
            axes.plot(result['times'], result['states'][i], color=c[i])

        if result['references'][i] != 0.0:
            axes.axhline(result['references'][i], linestyle='--', color=c[i])
            axes.axhline(result['references'][i] -
                         result['thresholds'][i], linestyle='-.', color=c[i])
            axes.axhline(result['references'][i] +
                         result['thresholds'][i], linestyle='-.', color=c[i])
            axes.axhline(result['references'][i] +
                         result['overshoots'][i], linestyle=':', color=c[i])
        if result['risetimes'][i] > 0.0:
            axes.axvline(result['risetimes'][i], linestyle='--', color=c[i])
        if result['settletimes'][i] > 0.0:
            axes.axvline(result['settletimes'][i], linestyle='-.', color=c[i])

    if legends:
        axes.legend()

    if title:
        axes.set_title(title)

    axes.set_xlim(result['times'][0], result['times'][-1])
    axes.figure.tight_layout()


def figure(w: float, h: float):
    dpi = 90
    print((w,h))
    return plt.figure(figsize=(w/dpi, h/dpi), dpi=dpi)


def axes(w: float, h: float):
    fig = figure(w, h)
    return fig.add_subplot(1, 1, 1)
