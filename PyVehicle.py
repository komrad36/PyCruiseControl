# *******************************************************************
# PyVehicle.py
# Cruise Control
# Kareem Omar
# Student Researcher, CSPAR - UAH Class of 2017
#
# 8/30/2015
# This program is entirely my own work.
# *******************************************************************

# The Vehicle class propagates the physics of a traveling vehicle.
# It does not handle PID control in any way that's contained
# in the PID module. This modular design means the PID controller
# is not dependent on the kind of vehicle, and vice versa.

class Vehicle:
        t = 0 # initial time [s]
        h = 0 # physics time step [s]
        v = 0 # initial v (modify w/ constructor) [m/s]
        steps = 0 # keep track of physics steps taken
        cur_pwr = 0 # current engine output [kW]
        min_pwr = 0 # min allowed power [kW]
        max_pwr = 149000 # max allowed power @ wheels [kW]
        m = 1542 # [kg]
        g = 9.81 # [m/s^2]
        A = 2 # [m^2]
        c_d = 0.34
        #c_d = 0.24 # it's a Tesla Model S...
        rho = 1.2041 # [kg/m^3]

        def __init__(this, v, h):
            this.v = v
            this.h = h

        # add hills here!
        def dh_dt(this, t, v):
            return 0
            #return 0.05*sin(t/5)

        # energy at time t is kinetic plus potential
        # energy loss is frictional (aerodrag)
        # energy input is engine power
        # E = 1/2 * m * v^2 + m * g * h + <accumulated drag losses> - <energy restored by engine> = const
        # dE/dt = mv(dv/dt) + mg(dh/dt) + 1/2pv^3a*C_d - engine = 0
        # mv(dv/dt) = engine - mg(dh/dt) - 1/2pv^3a*c_d
        # dv/dt = (engine - mg(dh/dt) - 1/2pv^3a*c_d) / mv
        def dv_dt(this, t, v):
            return (this.cur_pwr - this.m * this.g * this.dh_dt(t, v) - 0.5 * this.rho * (v ** 3) * this.A * this.c_d) / (this.m * v)
        
        
        def dv_dt_nopower(this, t, v):
            return (-this.m * this.g * this.dh_dt(t, v) - 0.5 * this.rho * (v ** 3) * this.A * this.c_d) / (this.m * v)

        # RK4 propagator
        def stepPhysics(this):
            k_1 = this.dv_dt(this.t, this.v)
            k_2 = this.dv_dt(this.t + this.h * 0.5, this.v + this.h * 0.5 * k_1)
            k_3 = this.dv_dt(this.t + this.h * 0.5, this.v + this.h * 0.5 * k_2)
            k_4 = this.dv_dt(this.t + this.h, this.v + this.h * k_3)
            this.t = this.h * this.steps
            this.steps += 1
            this.v += this.h / 6.0 * (k_1 + 2.0 * k_2 + 2.0 * k_3 + k_4)
            return this.v