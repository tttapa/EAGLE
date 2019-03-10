from py_drone_module import DronePlottable


def update_plot(plottable: DronePlottable, lines: dict):
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
