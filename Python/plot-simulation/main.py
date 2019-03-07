from py_drone_module import Drone, DroneReference, DroneReferenceFunction, AdaptiveODEOptions
from DroneParamsAndMatricesBuilder import buildDroneParamsAndMatrices
from numpy import array, diag
from math import pi
import Reference
import DronePlotter
from matplotlib.widgets import Slider
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1.inset_locator import inset_axes

dp = buildDroneParamsAndMatrices()
d = Drone(dp)

x0 = d.getStableState()

Q_att = diag((139.6245112700232,
              139.6245112700232,
              15.2811761590895,
              1.1505204155597211,
              1.1505204155597211,
              0.1209919487616804,
              9.976475759487083e-08,
              9.976475759487083e-08,
              9.976475759487083e-09))
R_att = diag((8, 8, 8))
K_pi_alt = array((0.0, 0.9, 0.5, -0.1))
maxIntegralInfluence = 0.1

controller = d.getController(Q_att, R_att, K_pi_alt, maxIntegralInfluence)

ref_function = DroneReferenceFunction(Reference.get_reference)

odeopt = AdaptiveODEOptions()
odeopt.t_start = 0
odeopt.t_end = 16
odeopt.epsilon = 1e-6
odeopt.h_start = 1e-6
odeopt.h_min = 1e-8
odeopt.maxiter = int(1e6)

result = d.simulate(controller, ref_function, x0, odeopt)
fig, lines, gridspec = DronePlotter.plot(result, extra_subplots_below=1)
ax_m = fig.add_subplot(gridspec[5, :])
slider_m = Slider(ax_m, 'm', 0.5, 5.5, valinit=dp.m, valstep=0.1)

def update(_):
    dp = buildDroneParamsAndMatrices(m=slider_m.val)
    print(dp.m)
    d = Drone(dp)
    controller = d.getController(Q_att, R_att, K_pi_alt, maxIntegralInfluence)
    result = d.simulate(controller, ref_function, x0, odeopt)
    DronePlotter.update_plot(lines, result)
    fig.canvas.draw_idle()


slider_m.on_changed(update)
DronePlotter.show(fig)