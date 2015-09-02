# *******************************************************************
# PyPID.py
# Cruise Control
# Kareem Omar
# Student Researcher, CSPAR - UAH Class of 2017
#
# 8/30/2015
# This program is entirely my own work.
# *******************************************************************

# The PID class contains a modified (see below) PID controller.
# It does not contain vehicle physics at all.
# This modular design means the PID controller
# is not dependent on the kind of vehicle, and vice versa.

class PID:
        # We expect fewer calls to the controller
        # than to physics engines.  Separately
        # keep a previous velocity and power for finite diff.
        prev_v = 0
        prev_pwr = 0
                    
        int_e = 0 # error integrator
        K_P = 0 # proportional gain
        K_I = 0 # integral gain
        K_D = 0 # derivative gain
        
        # Floor and ceiling of integral term to prevent
        # massive under/overshoots due to slow engine response or
        # slow drag-only deceleration
        min_I = 0
        max_I = 20
            
        # Max rates of integral term accum. in both neg and pos
        min_delta_I = -0.18
        max_delta_I = 1.45
    
        # Max power changes per dt [W]
        min_delta_pwr = -28000
        max_delta_pwr = 28000

        def __init__(this, K_P, K_I, K_D):
            this.K_P = K_P
            this.K_I = K_I
            this.K_D = K_D

        def stepControl(this, veh, set_point, dt):
            e = set_point - veh.v

            dv_dt = (veh.v - this.prev_v) / dt
            this.prev_v = veh.v
            de_dt = -dv_dt
            # This is normally ONLY TRUE if the set point
            # remains constant.  In this application it does not
            # however, we do not want the derivative corrector
            # producing an infinite power request if the set point
            # changes instantaneously.  Thus we deliberately ignore
            # set point changes and only respond to dv/dt.
            # de_dt = d(set_point - v)/dt = -dv/dt.

            # Prevent excessive over/undershoots by limiting
            # the RATE at which the integral term accumulates.
            # This is a *modification* of canonical PID.
            # It greatly improves performance by allowing
            # a much higher integral constant than would
            # otherwise be possible without causing enormous
            # over/undershoots, thus improving controller
            # response.
            if e > this.max_delta_I:
                this.int_e += dt * this.max_delta_I  
            elif e < this.min_delta_I:
                this.int_e += dt * this.min_delta_I    
            else:
                this.int_e += dt * e
            
            # Also prevent excessive over/undershoots by limiting
            # the integral term itself.
            # Specifically: the controller does not slam the brakes to
            # to decelerate (i.e.  min power is 0, not negative),
            # so this is not a truly linear system.  The car
            # may not have extreme acceleration and decelerates very
            # slowly, which would otherwise result in a huge build-up
            # of historical error.
            if this.int_e < this.min_I:
                this.int_e = this.min_I
            if this.int_e > this.max_I:
                this.int_e = this.max_I

            pwr = this.K_P * e + this.K_I * this.int_e + this.K_D * de_dt
            if pwr < veh.min_pwr:
                pwr = veh.min_pwr
            if pwr > veh.max_pwr:
                pwr = veh.max_pwr

            if pwr - this.prev_pwr > (this.max_delta_pwr * dt):
                pwr = this.prev_pwr + (this.max_delta_pwr * dt)
            if pwr - this.prev_pwr < (this.min_delta_pwr * dt):
                pwr = this.prev_pwr + (this.min_delta_pwr * dt)
            
            veh.cur_pwr = this.prev_pwr = pwr