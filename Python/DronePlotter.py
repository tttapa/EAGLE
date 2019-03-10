import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from py_drone_module import DronePlottable

matplotlib.rcParams['lines.linewidth'] = 0.9


def plot(plottable: DronePlottable, vertical: bool = False, w: float = 1920,
         h: float = 1080, colorset: int = 0, title: str = ""):

    if (w is None or h is None):
        fig = plt.figure()
    else:
        dpi = 90
        fig = plt.figure(figsize=(w/dpi, h/dpi), dpi=dpi)

    if vertical:
        grid = (10, 1)
        gs = fig.add_gridspec(*grid)

        ax_ref_ori = fig.add_subplot(gs[0, 0])
        ax_ori = fig.add_subplot(gs[1, 0])
        ax_ang_vel = fig.add_subplot(gs[2, 0])
        ax_torque = fig.add_subplot(gs[3, 0])
        ax_torque_ctrl = fig.add_subplot(gs[4, 0])

        ax_ref_pos = fig.add_subplot(gs[5, 0])
        ax_pos = fig.add_subplot(gs[6, 0])
        ax_lin_vel = fig.add_subplot(gs[7, 0])
        ax_thrust = fig.add_subplot(gs[8, 0])
        ax_thrust_ctrl = fig.add_subplot(gs[9, 0])
    else:
        grid = (5, 2)
        gs = fig.add_gridspec(*grid)

        ax_ref_ori = fig.add_subplot(gs[0, 0])
        ax_ref_pos = fig.add_subplot(gs[0, 1])

        ax_ori = fig.add_subplot(gs[1, 0])
        ax_pos = fig.add_subplot(gs[1, 1])

        ax_ang_vel = fig.add_subplot(gs[2, 0])
        ax_lin_vel = fig.add_subplot(gs[2, 1])

        ax_torque = fig.add_subplot(gs[3, 0])
        ax_thrust = fig.add_subplot(gs[3, 1])

        ax_torque_ctrl = fig.add_subplot(gs[4, 0])
        ax_thrust_ctrl = fig.add_subplot(gs[4, 1])

    colors = [
        {'x': 'red', 'y': 'green', 'z': 'blue'},
        {'x': 'tomato', 'y': 'lightgreen', 'z': 'steelblue'},
    ]

    c = colors[colorset]

    # TODO: I don't know if the order of x, y, z for Euler angles is correct.

    if title:
        fig.suptitle(title)

    data = DronePlottable(plottable).toDict()
    time = data['time']
    dtime = data['dtime']

    lines = {
        'reference_orientation': dict(),
        'reference_position': dict(),
        'orientation': dict(),
        'orientation_estimate': dict(),
        'position': dict(),
        'position_estimate': dict(),
        'angular_velocity': dict(),
        'angular_velocity_estimate': dict(),
        'linear_velocity': dict(),
        'linear_velocity_estimate': dict(),
        'torque_motor_velocity': dict(),
        'torque_motor_velocity_estimate': dict(),
        'thrust_motor_velocity': dict(),
        'thrust_motor_velocity_estimate': dict(),
        'torque_control': dict(),
        'thrust_control': dict(),
    }

    # ROW 0
    lines['reference_orientation']['x'], = ax_ref_ori.plot(
        dtime, data['reference_orientation']['x'], color=c['x'], label='x')
    lines['reference_orientation']['y'], = ax_ref_ori.plot(
        dtime, data['reference_orientation']['y'], color=c['y'], label='y')
    lines['reference_orientation']['z'], = ax_ref_ori.plot(
        dtime, data['reference_orientation']['z'], color=c['z'], label='z')
    ax_ref_ori.set_title("Reference orientation")
    ax_ref_ori.set_ylabel("Euler Angles [$\\mathrm{rad}$]")
    ax_ref_ori.set_xlim(time[0], time[-1])
    if not data['includes_estimated_states']:
        ax_ref_ori.legend()
    #
    #
    lines['reference_position']['x'], = ax_ref_pos.plot(
        dtime, data['reference_position']['x'], color=c['x'])
    lines['reference_position']['y'], = ax_ref_pos.plot(
        dtime, data['reference_position']['y'], color=c['y'])
    lines['reference_position']['z'], = ax_ref_pos.plot(
        dtime, data['reference_position']['z'], color=c['z'])
    ax_ref_pos.set_title("Reference position")
    ax_ref_pos.set_ylabel("Position [$m$]")
    ax_ref_pos.set_xlim(time[0], time[-1])
    #
    #
    # ROW 1
    lines['orientation']['x'], = ax_ori.plot(
        time, data['orientation']['x'], color=c['x'], label='x')
    lines['orientation']['y'], = ax_ori.plot(
        time, data['orientation']['y'], color=c['y'], label='y')
    lines['orientation']['z'], = ax_ori.plot(
        time, data['orientation']['z'], color=c['z'], label='z')
    if data['includes_estimated_states']:
        lines['orientation_estimate']['x'], = ax_ori.plot(
            dtime, data['orientation_estimate']['x'], '--', color=c['x'],
            label='x (est.)')
        lines['orientation_estimate']['y'], = ax_ori.plot(
            dtime, data['orientation_estimate']['y'], '--', color=c['y'],
            label='y (est.)')
        lines['orientation_estimate']['z'], = ax_ori.plot(
            dtime, data['orientation_estimate']['z'], '--', color=c['z'],
            label='z (est.)')
        ax_ori.legend()
    ax_ori.set_title("Orientation")
    ax_ori.set_ylabel("Euler Angles [$\\mathrm{rad}$]")
    ax_ori.set_xlim(time[0], time[-1])
    #
    #
    lines['position']['x'], = ax_pos.plot(
        time, data['position']['x'], color=c['x'])
    lines['position']['y'], = ax_pos.plot(
        time, data['position']['y'], color=c['y'])
    lines['position']['z'], = ax_pos.plot(
        time, data['position']['z'], color=c['z'])
    if data['includes_estimated_states']:
        lines['position_estimate']['x'], = ax_pos.plot(
            dtime, data['position_estimate']['x'], '--', color=c['x'])
        lines['position_estimate']['y'], = ax_pos.plot(
            dtime, data['position_estimate']['y'], '--', color=c['y'])
        lines['position_estimate']['z'], = ax_pos.plot(
            dtime, data['position_estimate']['z'], '--', color=c['z'])
    ax_pos.set_title("Position")
    ax_pos.set_ylabel("Position [$m$]")
    ax_pos.set_xlim(time[0], time[-1])
    #
    #
    # ROW 2
    lines['angular_velocity']['x'], = ax_ang_vel.plot(
        time, data['angular_velocity']['x'], color=c['x'])
    lines['angular_velocity']['y'], = ax_ang_vel.plot(
        time, data['angular_velocity']['y'], color=c['y'])
    lines['angular_velocity']['z'], = ax_ang_vel.plot(
        time, data['angular_velocity']['z'], color=c['z'])
    if data['includes_estimated_states']:
        lines['angular_velocity_estimate']['x'], = ax_ang_vel.plot(
            dtime, data['angular_velocity_estimate']['x'], '--', color=c['x'])
        lines['angular_velocity_estimate']['y'], = ax_ang_vel.plot(
            dtime, data['angular_velocity_estimate']['y'], '--', color=c['y'])
        lines['angular_velocity_estimate']['z'], = ax_ang_vel.plot(
            dtime, data['angular_velocity_estimate']['z'], '--', color=c['z'])
    ax_ang_vel.set_title("Angular velocity")
    ax_ang_vel.set_ylabel("Angular velocity [$\\mathrm{rad}/s$]")
    ax_ang_vel.set_xlim(time[0], time[-1])
    #
    #
    lines['linear_velocity']['x'], = ax_lin_vel.plot(
        time, data['linear_velocity']['x'], color=c['x'])
    lines['linear_velocity']['y'], = ax_lin_vel.plot(
        time, data['linear_velocity']['y'], color=c['y'])
    lines['linear_velocity']['z'], = ax_lin_vel.plot(
        time, data['linear_velocity']['z'], color=c['z'])
    if data['includes_estimated_states']:
        lines['linear_velocity_estimate']['x'], = ax_lin_vel.plot(
            dtime, data['linear_velocity_estimate']['x'], '--', color=c['x'])
        lines['linear_velocity_estimate']['y'], = ax_lin_vel.plot(
            dtime, data['linear_velocity_estimate']['y'], '--', color=c['y'])
        lines['linear_velocity_estimate']['z'], = ax_lin_vel.plot(
            dtime, data['linear_velocity_estimate']['z'], '--', color=c['z'])
    ax_lin_vel.set_title("Velocity")
    ax_lin_vel.set_ylabel("Velocity [$m/s$]")
    ax_lin_vel.set_xlim(time[0], time[-1])
    #
    #
    # ROW 3
    lines['torque_motor_velocity']['x'], = ax_torque.plot(
        time, data['torque_motor_velocity']['x'], color=c['x'])
    lines['torque_motor_velocity']['y'], = ax_torque.plot(
        time, data['torque_motor_velocity']['y'], color=c['y'])
    lines['torque_motor_velocity']['z'], = ax_torque.plot(
        time, data['torque_motor_velocity']['z'], color=c['z'])
    if data['includes_estimated_states']:
        lines['torque_motor_velocity_estimate']['x'], = ax_torque.plot(
            dtime, data['torque_motor_velocity_estimate']['x'], '--', 
            color=c['x'])
        lines['torque_motor_velocity_estimate']['y'], = ax_torque.plot(
            dtime, data['torque_motor_velocity_estimate']['y'], '--', 
            color=c['y'])
        lines['torque_motor_velocity_estimate']['z'], = ax_torque.plot(
            dtime, data['torque_motor_velocity_estimate']['z'], '--', 
            color=c['z'])
    ax_torque.set_title("Torque motor velocity")
    ax_torque.set_ylabel("Angular velocity [$?$]")
    ax_torque.set_xlim(time[0], time[-1])
    #
    #
    lines['thrust_motor_velocity']['z'], = ax_thrust.plot(
        time, data['thrust_motor_velocity']['z'], color=c['z'])
    if data['includes_estimated_states']:
        lines['thrust_motor_velocity_estimate']['z'], = ax_thrust.plot(
            dtime, data['thrust_motor_velocity_estimate']['z'], '--', 
            color=c['z'])
    ax_thrust.set_title("Thrust motor velocity")
    ax_thrust.set_ylabel("Angular velocity [$?$]")
    ax_thrust.set_xlim(time[0], time[-1])
    ax_thrust.margins(y=0.5)
    #
    #
    # ROW 4
    lines['torque_control']['x'], = ax_torque_ctrl.plot(
        dtime, data['torque_control']['x'], color=c['x'])
    lines['torque_control']['y'], = ax_torque_ctrl.plot(
        dtime, data['torque_control']['y'], color=c['y'])
    lines['torque_control']['z'], = ax_torque_ctrl.plot(
        dtime, data['torque_control']['z'], color=c['z'])
    ax_torque_ctrl.set_title("Torque motor control")
    ax_torque_ctrl.set_ylabel("Control signal [-]")
    ax_torque_ctrl.set_xlabel("Time [$s$]")
    ax_torque_ctrl.set_xlim(time[0], time[-1])
    #
    #
    lines['thrust_control']['z'], = ax_thrust_ctrl.plot(
        dtime, data['thrust_control']['z'], color=c['z'])
    ax_thrust_ctrl.set_title("Thrust motor control")
    ax_thrust_ctrl.set_ylabel("Control signal [-]")
    ax_thrust_ctrl.set_xlabel("Time [$s$]")
    ax_thrust_ctrl.set_xlim(time[0], time[-1])
    ax_thrust_ctrl.margins(y=0.5)

    fig.tight_layout()
    return fig, lines

from py_drone_module import DronePlottable


def update_plot(lines: dict, plottable: DronePlottable):
    data = DronePlottable(plottable).toDict()
    time = data['time']
    dtime = data['dtime']

    # ( *)lines(\['\w+'\]\['\w+'\])\s*=\s*\w+\.plot\(\s*(\w+),\s+([^,]+)[^)]+\)
    # $1lines$2.set_ydata($4)\n$1lines$2.set_xdata($3)

    # ROW 0
    lines['reference_orientation']['x'].set_ydata(
        data['reference_orientation']['x'])
    lines['reference_orientation']['x'].set_xdata(dtime)
    lines['reference_orientation']['y'].set_ydata(
        data['reference_orientation']['y'])
    lines['reference_orientation']['y'].set_xdata(dtime)
    lines['reference_orientation']['z'].set_ydata(
        data['reference_orientation']['z'])
    lines['reference_orientation']['z'].set_xdata(dtime)
    #
    #
    lines['reference_position']['x'].set_ydata(data['reference_position']['x'])
    lines['reference_position']['x'].set_xdata(dtime)
    lines['reference_position']['y'].set_ydata(data['reference_position']['y'])
    lines['reference_position']['y'].set_xdata(dtime)
    lines['reference_position']['z'].set_ydata(data['reference_position']['z'])
    lines['reference_position']['z'].set_xdata(dtime)
    #
    #
    # ROW 1
    lines['orientation']['x'].set_ydata(data['orientation']['x'])
    lines['orientation']['x'].set_xdata(time)
    lines['orientation']['y'].set_ydata(data['orientation']['y'])
    lines['orientation']['y'].set_xdata(time)
    lines['orientation']['z'].set_ydata(data['orientation']['z'])
    lines['orientation']['z'].set_xdata(time)
    if data['includes_estimated_states']:
        lines['orientation_estimate']['x'].set_ydata(
            data['orientation_estimate']['x'])
        lines['orientation_estimate']['x'].set_xdata(dtime)
        lines['orientation_estimate']['y'].set_ydata(
            data['orientation_estimate']['y'])
        lines['orientation_estimate']['y'].set_xdata(dtime)
        lines['orientation_estimate']['z'].set_ydata(
            data['orientation_estimate']['z'])
        lines['orientation_estimate']['z'].set_xdata(dtime)
    #
    #
    lines['position']['x'].set_ydata(data['position']['x'])
    lines['position']['x'].set_xdata(time)
    lines['position']['y'].set_ydata(data['position']['y'])
    lines['position']['y'].set_xdata(time)
    lines['position']['z'].set_ydata(data['position']['z'])
    lines['position']['z'].set_xdata(time)
    if data['includes_estimated_states']:
        lines['position_estimate']['x'].set_ydata(
            data['position_estimate']['x'])
        lines['position_estimate']['x'].set_xdata(dtime)
        lines['position_estimate']['y'].set_ydata(
            data['position_estimate']['y'])
        lines['position_estimate']['y'].set_xdata(dtime)
        lines['position_estimate']['z'].set_ydata(
            data['position_estimate']['z'])
        lines['position_estimate']['z'].set_xdata(dtime)
    #
    #
    # ROW 2
    lines['angular_velocity']['x'].set_ydata(data['angular_velocity']['x'])
    lines['angular_velocity']['x'].set_xdata(time)
    lines['angular_velocity']['y'].set_ydata(data['angular_velocity']['y'])
    lines['angular_velocity']['y'].set_xdata(time)
    lines['angular_velocity']['z'].set_ydata(data['angular_velocity']['z'])
    lines['angular_velocity']['z'].set_xdata(time)
    if data['includes_estimated_states']:
        lines['angular_velocity_estimate']['x'].set_ydata(
            data['angular_velocity_estimate']['x'])
        lines['angular_velocity_estimate']['x'].set_xdata(dtime)
        lines['angular_velocity_estimate']['y'].set_ydata(
            data['angular_velocity_estimate']['y'])
        lines['angular_velocity_estimate']['y'].set_xdata(dtime)
        lines['angular_velocity_estimate']['z'].set_ydata(
            data['angular_velocity_estimate']['z'])
        lines['angular_velocity_estimate']['z'].set_xdata(dtime)
    #
    #
    lines['linear_velocity']['x'].set_ydata(data['linear_velocity']['x'])
    lines['linear_velocity']['x'].set_xdata(time)
    lines['linear_velocity']['y'].set_ydata(data['linear_velocity']['y'])
    lines['linear_velocity']['y'].set_xdata(time)
    lines['linear_velocity']['z'].set_ydata(data['linear_velocity']['z'])
    lines['linear_velocity']['z'].set_xdata(time)
    if data['includes_estimated_states']:
        lines['linear_velocity_estimate']['x'].set_ydata(
            data['linear_velocity_estimate']['x'])
        lines['linear_velocity_estimate']['x'].set_xdata(dtime)
        lines['linear_velocity_estimate']['y'].set_ydata(
            data['linear_velocity_estimate']['y'])
        lines['linear_velocity_estimate']['y'].set_xdata(dtime)
        lines['linear_velocity_estimate']['z'].set_ydata(
            data['linear_velocity_estimate']['z'])
        lines['linear_velocity_estimate']['z'].set_xdata(dtime)
    #
    #
    # ROW 3
    lines['torque_motor_velocity']['x'].set_ydata(
        data['torque_motor_velocity']['x'])
    lines['torque_motor_velocity']['x'].set_xdata(time)
    lines['torque_motor_velocity']['y'].set_ydata(
        data['torque_motor_velocity']['y'])
    lines['torque_motor_velocity']['y'].set_xdata(time)
    lines['torque_motor_velocity']['z'].set_ydata(
        data['torque_motor_velocity']['z'])
    lines['torque_motor_velocity']['z'].set_xdata(time)
    if data['includes_estimated_states']:
        lines['torque_motor_velocity_estimate']['x'].set_ydata(
            data['torque_motor_velocity_estimate']['x'])
        lines['torque_motor_velocity_estimate']['x'].set_xdata(dtime)
        lines['torque_motor_velocity_estimate']['y'].set_ydata(
            data['torque_motor_velocity_estimate']['y'])
        lines['torque_motor_velocity_estimate']['y'].set_xdata(dtime)
        lines['torque_motor_velocity_estimate']['z'].set_ydata(
            data['torque_motor_velocity_estimate']['z'])
        lines['torque_motor_velocity_estimate']['z'].set_xdata(dtime)
    #
    #
    lines['thrust_motor_velocity']['z'].set_ydata(
        data['thrust_motor_velocity']['z'])
    lines['thrust_motor_velocity']['z'].set_xdata(time)
    if data['includes_estimated_states']:
        lines['thrust_motor_velocity_estimate']['z'].set_ydata(
            data['thrust_motor_velocity_estimate']['z'])
        lines['thrust_motor_velocity_estimate']['z'].set_xdata(dtime)
    #
    #
    # ROW 4
    lines['torque_control']['x'].set_ydata(data['torque_control']['x'])
    lines['torque_control']['x'].set_xdata(dtime)
    lines['torque_control']['y'].set_ydata(data['torque_control']['y'])
    lines['torque_control']['y'].set_xdata(dtime)
    lines['torque_control']['z'].set_ydata(data['torque_control']['z'])
    lines['torque_control']['z'].set_xdata(dtime)
    #
    #
    lines['thrust_control']['z'].set_ydata(data['thrust_control']['z'])
    lines['thrust_control']['z'].set_xdata(dtime)



def plot_attitude(time, dtime, states: dict, w: float, h: float, colorset: int, title: str):
    dpi = 90
    fig, (
        ax_ref_ori,
        ax_ori,
        ax_ang_vel,
        ax_torque,
        ax_torque_ctrl
    ) = plt.subplots(nrows=5, ncols=1, figsize=(w/dpi, h/dpi), dpi=dpi)

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
    # ROW 1
    ax_ori.plot(time, states['orientation']['x'], color=c['x'])
    ax_ori.plot(time, states['orientation']['y'], color=c['y'])
    ax_ori.plot(time, states['orientation']['z'], color=c['z'])
    ax_ori.set_title("Orientation")
    ax_ori.set_ylabel("Euler Angles [$\\mathrm{rad}$]")
    ax_ori.set_xlim(time[0], time[-1])
    #
    # ROW 2
    ax_ang_vel.plot(time, states['angular_velocity']['x'], color=c['x'])
    ax_ang_vel.plot(time, states['angular_velocity']['y'], color=c['y'])
    ax_ang_vel.plot(time, states['angular_velocity']['z'], color=c['z'])
    ax_ang_vel.set_title("Angular velocity")
    ax_ang_vel.set_ylabel("Angular velocity [$\\mathrm{rad}/s$]")
    ax_ang_vel.set_xlim(time[0], time[-1])
    #
    # ROW 3
    ax_torque.plot(time, states['torque_motor_velocity']['x'], color=c['x'])
    ax_torque.plot(time, states['torque_motor_velocity']['y'], color=c['y'])
    ax_torque.plot(time, states['torque_motor_velocity']['z'], color=c['z'])
    ax_torque.set_title("Torque motor velocity")
    ax_torque.set_ylabel("Angular velocity [$?$]")
    ax_torque.set_xlim(time[0], time[-1])
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

    fig.tight_layout()
    return fig


def plot_step_analyzer(axes: matplotlib.axes.Axes, result: dict,
                       title: str = "Step response", legends: list = [],
                       colorset: int = 0):
    colors = [
        ('red', 'green', 'blue'),  # TODO: add more colors
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
    return plt.figure(figsize=(w/dpi, h/dpi), dpi=dpi)


def axes(w: float, h: float):
    fig = figure(w, h)
    return fig.add_subplot(1, 1, 1)


def close(figure: matplotlib.figure.Figure):
    plt.close(fig=figure)


def show(fig: matplotlib.figure.Figure = None):
    if fig is not None:
        plt.show(fig)
    else:
        plt.show()


def save(fig: matplotlib.figure.Figure, filename: str):
    fig.savefig(filename)
