import numpy as np

p = drone.DroneParamsAndMatrices()
p.load("/home/pieter/PO-EAGLE/Groups/ANC/Cleanup-Pieter/Code-Generators/ParamsAndMatrices/Output")
d = drone.Drone(p)
u = np.array((0.6, 0.5, 0.5, 0.6))
x0 = d.getStableState()
print(d(x0.asColVector(), u))
