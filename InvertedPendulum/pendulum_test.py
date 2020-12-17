import time
from numpy import *
from pynput import keyboard
import can
from tinymovr import Tinymovr
from tinymovr.iface.can import CAN, guess_channel

global goon,zeroFlag,nrjFlag
goon = True
zeroFlag = False
nrjFlag = False
T0 = 23.3/30
w0 = 2*pi/T0

channel = guess_channel(bustype_hint='slcan')
print(channel)
bus = can.Bus(bustype="slcan",channel=channel)

iface = CAN(bus)
print(iface)
time.sleep(1)
tm = Tinymovr(node_id=1, iface=iface)

def on_press(key):
    global goon,zeroFlag,nrjFlag
    # print(key)
    if key == keyboard.Key.esc:
        goon = False
    if key == keyboard.Key.right:
        tm.current_control()
        tm.set_cur_setpoint(5)
    if key == keyboard.Key.left:
        tm.current_control()
        tm.set_cur_setpoint(-5)
    if key == keyboard.Key.down:
        tm.velocity_control()
        tm.set_vel_setpoint(0)
    if key == keyboard.Key.up:
        tm.position_control()
        tm.set_pos_setpoint(theta0+4096)
    if key == keyboard.Key.home:
        zeroFlag = True
    if key == keyboard.Key.space:
        nrjFlag = True
    


def on_release(key):
    global goon,zeroFlag,nrjFlag
    # print(key)
    tm.current_control()
    tm.set_cur_setpoint(0)
    nrjFlag = False

if tm.motor_info['calibrated'] == 0:
    tm.calibrate()
    time.sleep(10)
    tm.save_config()



tm.set_limits(velocity=500000.0,current=10.0)
tm.set_gains(position=50.0, velocity=0.001)

listener = keyboard.Listener(on_press=on_press,on_release=on_release)
listener.start()

# amortissement
print("Amortissement du pendule.")
tm.velocity_control()
tm.set_vel_setpoint(0)
tm.current_control()
time.sleep(5)

# mise à zero
print("Mise à zero.")
theta0 = tm.encoder_estimates['position']
thetap0 = tm.encoder_estimates['velocity']
nrj0=-w0*w0

# algo
print("GO")
while goon:
    theta = pi*(tm.encoder_estimates['position']-theta0)/4096
    thetap = pi*(tm.encoder_estimates['velocity']-thetap0)/4096
    # print("{:.2f}\t{:.2f}".format(theta,thetap))
    nrj = 0.5*thetap*thetap-w0*w0*cos(theta)
    # print("{:.1f}".format(nrj))
    # print("{:.0f}".format(tm.encoder_estimates['position']-theta0))
    

    if zeroFlag:
        theta0 = tm.encoder_estimates['position']
        thetap0 = tm.encoder_estimates['velocity']
        zeroFlag = False
    if nrjFlag:
        if theta>95*pi/100:
            tm.position_control()
            tm.set_pos_setpoint(theta0+4096)
        else:
            tm.current_control()
            error = nrj-(-nrj0+9)
            torque=-0.05*thetap*error
            torque=max(min(torque,5.0),-5.0)
            print("{:.2f}\t>{:.1f}".format(torque,nrj))
            tm.set_cur_setpoint(torque)

    time.sleep(0.01)
