PyCruiseControl is a modified divorced PID controller applied to car cruise control and accompanying physics simulation and visualizations.

This controller demonstrates the principles of optimizing PID (or PI in this case - the derivative term
is not necessary for this application) control for a real-world application and propagates the physics of hills
and aerodrag (things that can affect a car's kinetic energy) to provide a simulation of the controller. These parameters are adjustable, including a function for providing custom terrain. The system also plots the resulting velocities as well as requested engine power.

There is a MATLAB port also available on my GitHub at https://github.com/komrad36.

The file PyCruiseControl.pyw is the run script; the other two are helper modules containing the PID controller
and the vehicle physics simulator.

The only dependency is matplotlib for plotting support.

Two screenshots in this directory illustrate example plots of a particular setpoint configuration.