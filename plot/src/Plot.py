import matplotlib.pyplot as plt
import numpy as np
from math import pi


def plot(time, dtime, states):
    fig, (
        #
        (ax_ref_ori,      ax_ref_pos),
        (ax_ori,          ax_pos),
        (ax_ang_vel,      ax_lin_vel),
        (ax_torque,       ax_thrust),
        (ax_torque_ctrl,  ax_thrust_ctrl)
        #
    ) = plt.subplots(nrows=5, ncols=2, figsize=(20, 10))

    # TODO: I don't know if the order of x, y, z for Euler angles is correct.

    # ROW 0
    ax_ref_ori.plot(
        dtime, states['reference_orientation']['x'], color='red', label='x')
    ax_ref_ori.plot(
        dtime, states['reference_orientation']['y'], color='green', label='y')
    ax_ref_ori.plot(
        dtime, states['reference_orientation']['z'], color='blue', label='z')
    ax_ref_ori.set_title("Reference orientation")
    ax_ref_ori.set_ylabel("Euler Angles [$\\mathrm{rad}$]")
    ax_ref_ori.set_xlim(time[0], time[-1])
    ax_ref_ori.legend()
    #
    #
    ax_ref_pos.plot(dtime, states['reference_position']['x'], color='red')
    ax_ref_pos.plot(dtime, states['reference_position']['y'], color='green')
    ax_ref_pos.plot(dtime, states['reference_position']['z'], color='blue')
    ax_ref_pos.set_title("Reference position")
    ax_ref_pos.set_ylabel("Position [$m$]")
    ax_ref_pos.set_xlim(time[0], time[-1])
    #
    #
    # ROW 1
    ax_ori.plot(time, states['orientation']['x'], color='red')
    ax_ori.plot(time, states['orientation']['y'], color='green')
    ax_ori.plot(time, states['orientation']['z'], color='blue')
    ax_ori.set_title("Orientation")
    ax_ori.set_ylabel("Euler Angles [$\\mathrm{rad}$]")
    ax_ori.set_xlim(time[0], time[-1])
    #
    #
    ax_pos.plot(time, states['position']['x'], color='red')
    ax_pos.plot(time, states['position']['y'], color='green')
    ax_pos.plot(time, states['position']['z'], color='blue')
    ax_pos.set_title("Position")
    ax_pos.set_ylabel("Position [$m$]")
    ax_pos.set_xlim(time[0], time[-1])
    #
    #
    # ROW 2
    ax_ang_vel.plot(time, states['angular_velocity']['x'], color='red')
    ax_ang_vel.plot(time, states['angular_velocity']['y'], color='green')
    ax_ang_vel.plot(time, states['angular_velocity']['z'], color='blue')
    ax_ang_vel.set_title("Angular velocity")
    ax_ang_vel.set_ylabel("Angular velocity [$\\mathrm{rad}/s$]")
    ax_ang_vel.set_xlim(time[0], time[-1])
    #
    #
    ax_lin_vel.plot(time, states['linear_velocity']['x'], color='red')
    ax_lin_vel.plot(time, states['linear_velocity']['y'], color='green')
    ax_lin_vel.plot(time, states['linear_velocity']['z'], color='blue')
    ax_lin_vel.set_title("Velocity")
    ax_lin_vel.set_ylabel("Velocity [$m/s$]")
    ax_lin_vel.set_xlim(time[0], time[-1])
    #
    #
    # ROW 3
    ax_torque.plot(time, states['torque_motor_velocity']['x'], color='red')
    ax_torque.plot(time, states['torque_motor_velocity']['y'], color='green')
    ax_torque.plot(time, states['torque_motor_velocity']['z'], color='blue')
    ax_torque.set_title("Torque motor velocity")
    ax_torque.set_ylabel("Angular velocity [$?$]")
    ax_torque.set_xlim(time[0], time[-1])
    #
    #
    ax_thrust.plot(time, states['thrust_motor_velocity']['z'], color='blue')
    ax_thrust.set_title("Thrust motor velocity")
    ax_thrust.set_ylabel("Angular velocity [$?$]")
    ax_thrust.set_xlim(time[0], time[-1])
    #
    #
    # ROW 4
    ax_torque_ctrl.plot(dtime, states['torque_control']['x'], color='red')
    ax_torque_ctrl.plot(dtime, states['torque_control']['y'], color='green')
    ax_torque_ctrl.plot(dtime, states['torque_control']['z'], color='blue')
    ax_torque_ctrl.set_title("Torque motor control")
    ax_torque_ctrl.set_ylabel("Control signal [-]")
    ax_torque_ctrl.set_xlabel("Time [$s$]")
    ax_torque_ctrl.set_xlim(time[0], time[-1])
    #
    #
    ax_thrust_ctrl.plot(dtime, states['thrust_control']['z'], color='blue')
    ax_thrust_ctrl.set_title("Thrust motor control")
    ax_thrust_ctrl.set_ylabel("Control signal [-]")
    ax_thrust_ctrl.set_xlabel("Time [$s$]")
    ax_thrust_ctrl.set_xlim(time[0], time[-1])

    plt.tight_layout()
    plt.show()
