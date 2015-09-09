# *******************************************************************
# PyCruiseControl.py
# Cruise Control
# 
# Kareem Omar
# Student Researcher, CSPAR - UAH Class of 2017
# 
# 8/30/2015
# This program is entirely my own work.
# *******************************************************************
# 
# Cruise Control is a PID controller (or PI - derivative
# term not required for this application), along with the
# required physics simulation (i.e. hills, drag) for demo purposes
# 
# This controller is optimized for normal car speeds, i.e. < 50 m/s.
# Of course, the controller may perform suboptimally far above this.
#
# Experiment with the parameters below, or perform advanced
# experimentation or modification of the PID system in PyPID.
#
# Add 'hills' with the dh_dt function in PyVehicle.

V_init = 25 # initial velocity [m/s]
maximize_plots = True
font_size = 24 # [pt]
h_physics = 0.05 # physics time step [s]
h_ref = 0.01 # reference (coasting) integrator time step [s]
physics_ticks_per_control_tick = 4 # call PID every x physics ticks
T_max = 200 # end time [s]

# PID gains:
K_P = 22000  # proportional gain
K_I = 2200   # integral gain
K_D = 0      # derivative gain

num_ticks = int(T_max / h_physics) + 1
# ------adjust this array to configure setpoints as desired------
#set_points = [V_init] * (num_ticks // 4) + [15] * (num_ticks // 7) + [29] * (num_ticks // 4)
#set_points = []
set_points = [V_init] * (num_ticks // 9) + [37] * (num_ticks // 6) + [34] * (num_ticks // 7) + [25] * (num_ticks // 3) + [10] * (num_ticks // 7)
# remainder of array:
if len(set_points) < num_ticks:
    set_points.extend([29] * (num_ticks - len(set_points)))

#----------------------------------------------------------------

# --------- END OF USER-CONFIGURABLE PARAMETERS ---------

import PyVehicle
import PyPID
import matplotlib.pyplot as plt
from math import ceil

veh = PyVehicle.Vehicle(V_init, h_physics)
pid = PyPID.PID(K_P, K_I, K_D)

# RK4 propagator
def RK4(dv_dt, t, dt, v):
    k_1 = dv_dt(t, v)
    k_2 = dv_dt(t + dt * 0.5, v + dt * 0.5 * k_1)
    k_3 = dv_dt(t + dt * 0.5, v + dt * 0.5 * k_2)
    k_4 = dv_dt(t + dt, v + dt * k_3)
    return v + dt / 6.0 * (k_1 + 2.0 * k_2 + 2.0 * k_3 + k_4)

# maximize plots if desired, on any backend
def maximizePlot():
    mng = plt.get_current_fig_manager()
    backend = plt.get_backend()
    if backend == 'TkAgg':
        mng.window.state('zoomed')
    elif backend == 'wxAgg':
        mng.frame.Maximize(True)
    elif backend == 'QT4Agg':
        mng.window.showMaximized()
    else:
        return False
    return True

# reference integrator with no engine power, i.e.  coasting
num_ref_ticks = int(T_max / h_ref)
# preallocate
T_nopower = [0] + [None] * num_ref_ticks
V_nopower = [V_init] + [None] * num_ref_ticks
i = 0
while T_nopower[i] < T_max:
    i += 1
    # floor vel at just over 0 so we are not annoyed by massive slides down hills
    V_nopower[i] = max(0.01, RK4(veh.dv_dt_nopower, T_nopower[i - 1], h_ref, V_nopower[i - 1]))
    T_nopower[i] = i * h_ref

# propagate physics and control
h_control = h_physics * physics_ticks_per_control_tick
# preallocate
T = [None] * num_ticks
V = [None] * num_ticks
pwr = [None] * num_ticks
for i in range(num_ticks):
    V[i] = veh.stepPhysics()
    T[i] = veh.t
    if not i % physics_ticks_per_control_tick:
        pid.stepControl(veh, set_points[i], h_control)
    
    pwr[i] = veh.cur_pwr

# plot
plt.rcParams['font.size'] = font_size

plt.figure('PID Cruise Control - Requested Engine Power')
if maximize_plots:
    maximizePlot()
plt.title('PID Cruise Control\nRequested Engine Power')
plt.xlabel('Time [s]')
plt.ylabel('Req. Power [kW]')
KW_PER_W = 1e-3
max_kw = veh.max_pwr * KW_PER_W
plt.axhline(max_kw, color='r', label='Maximum Engine Power', linewidth=1.5)
plt.plot(T, [elem * KW_PER_W for elem in pwr], color='b', linewidth=1.5)
plt.ylim(0, 20 * ceil(max_kw / 20) + 10)
plt.legend(loc='best')

plt.figure('PID Cruise Control - Velocities')
if maximize_plots:
    maximizePlot()
plt.title('PID Cruise Control\nVelocities')
plt.xlabel('Time [s]')
plt.ylabel('Speed [m/s]')
plt.plot(T, set_points, color='r', label='Set Point')
plt.plot(T, V, color='b', linewidth=2, label='Cruise Control')
plt.plot(T_nopower, V_nopower, color='g', linewidth=2, label='Coasting')
plt.ylim(0, 10 * ceil(max(V + set_points) / 10) + 5)
plt.legend(loc='best')

plt.show()