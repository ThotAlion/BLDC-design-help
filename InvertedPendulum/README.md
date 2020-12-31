# Torque limited actuated inverted pendulum
BLDC controlled by a Tinymovr controller allows to build and experiment a "classical control problem" such as the inverted pendulum. Most of time, it is done in simulation but we cannot "play with it". Such a motor can perform in torque control with a relatively high torque (with respect to classical DC brushed motor). The fact the torque is controlled using current control is also useful since the current is "limited by design" and it will be difficult to exceed a maximum current. The risk of big melting motor is prevented.

There are several setups for inverted pendulums such as cart-pole. Here, the setup is to fix the pendulum directly on the rotor of the brushless motor. But we limit the torque such as it is not possible to lift directly the pendulum to set it upright.

https://www.youtube.com/watch?v=rv7isctR4a8

The motor chosen here is a 2204 at 2300KV (about 10$) found in our old shielves, piloted by the high tech Tinymovr controller with CANable interface. The max current of such a motor is 15Amps which is below the maximal allowed current by Tinymovr. The advantage of Tinymovr is the integrated absolute magnetic encoder and the CANable robust and fast interface.

Therefore, this dynamical control test setup is very cheap and easy to make. The but control theory needed in the background is advanced.

We advise to follow the underactuated robotics course of Russ Tedrake 

Russ Tedrake. Underactuated Robotics: Algorithms for Walking, Running, Swimming, Flying, and Manipulation (Course Notes for MIT 6.832). Downloaded on december 2020 from http://underactuated.mit.edu/

This test bench jumps directly [here](http://underactuated.mit.edu/pend.html#section3)

## Software setup

The Python script runs Python 3 using :
- pygame for keyboard real-time control and display
- tinymovr official library
- numpy

The setup here is for an Ubuntu 18 WSL under Windows 10 with [VcXsrv X Server](http://vcxsrv.sourceforge.net)

Line 36, the channel, if your CANable USB interface is on port COM6, the Ubuntu 18 WSL will see it on /dev/ttyS6

## Python structure

This script is of course not an "industrial" one but the objective is to be very short and with few functions. The structure is a classical for real-time test bench :

- Introduction to instanciate all variables and interfaces
- Open an infinite loop at 100Hz
- Get all sensors  (including keyborad inputs)
- running a finite state machine (including safety conditions)
- running a control loop (here all is done inside Tinymovr board)
- send orders (including screen display)

The finite state machine has 4 states :
- 0 : the safety state with current controlled to 0Amps. This state is set each time a key is released (dead man safety)
- 1 : the damping state with velocity controlled to zero. Just push down arrow to damp the pendulum. In some seconds, the pendulum is stopped. (press down arrow)
- 2 : the most "exotic" state with mechanical energy control. the nrj variable is proportionnal to the total mechanical energy of the pendulum, the sum of potential energy and kinetic energy. Without any torque of air damping, this energy must be conserved. In this mode, the energy is controlled to reach nrj0, the energy of the still upright pendulum. (press space bar)
- 3 : Upright proportional controller. When the pendulum is close to the upright position with a relatively small speed, it is possible to control the pendulum upright very easily using internal Tinymovr control loop. (press up arrow)

Notice states 2 and 3 can switch between them, the condition is to be "almost upright".

You can leave the program by pressing ESC or click the right upper cross.

Press the "z" key to reset the zero of the pendulum (down)

Press the "r" key to hard reset the Tinymovr (the program will also be killed)

