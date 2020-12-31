from numpy import *
import pygame
import time
import can
from tinymovr import Tinymovr
from tinymovr.iface.can import CAN, guess_channel
from tinymovr.units import get_registry

# Definition of units for Pint
ureg = get_registry()
Q_ = ureg.Quantity
A = ureg.ampere
s = ureg.second
min = ureg.minute
rad = ureg.radian
deg = ureg.degree
turn = ureg.turn
degC = ureg.degC

# init variables
dt = 0.01
goon = True
modepygame = 0
# pendulum period
T0 = 23.3/30 #30 oscillations in 23.3s
# pendulum natural pulsation
w0 = 2*pi/T0*rad/s
nrj0=-w0*w0

# pygame init
pygame.init()
ecran = pygame.display.set_mode((1000, 1000))
font = pygame.font.SysFont(None, 48)

# Instanciate Tinymovr interface
# channel = guess_channel(bustype_hint='slcan')
channel='/dev/ttyS6'
can_bus = can.Bus(bustype='slcan',channel=channel,bitrate=1000000)
iface = CAN(can_bus)
tm = Tinymovr(node_id=1, iface=iface)

# initialise motor control gains and limits
tm.set_limits(velocity=2000*turn/min, current=10.0*A)
tm.set_gains(position=50.0, velocity=0.001)
print(tm.motor_info)
print(tm.device_info)

# set the zero (assuming the pendulum is downward and still, else, press "z" key)
theta0 = tm.encoder_estimates.position.to(rad)
thetap0 = tm.encoder_estimates.velocity.to(rad/s)

# control loop (100Hz)
while goon:
    # recup keyboard input
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            goon = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                goon = False
            elif event.key == pygame.K_DOWN:
                print ("bas")
                modepygame = 1
            elif event.key == pygame.K_UP:
                print ("haut")
                modepygame = 3
            elif event.key == pygame.K_LEFT:
                print ("gauche")
            elif event.key == pygame.K_RIGHT:
                print ("droite")
            elif event.key == pygame.K_z:
                print ("zero")
                theta0 = tm.encoder_estimates.position.to(rad)
                thetap0 = tm.encoder_estimates.velocity.to(rad/s)
            elif event.key == pygame.K_r:
                print ("reset")
                tm.reset()
            elif event.key == pygame.K_SPACE:
                print ("balance")
                modepygame = 2
        elif event.type == pygame.KEYUP:
            modepygame = 0

    # recup motor sensors and convert to S.I.
    theta = tm.encoder_estimates.position.to(rad)-theta0
    thetap = tm.encoder_estimates.velocity.to(rad/s)-thetap0
    nrj = 0.5*thetap*thetap-w0*w0*cos(theta)

    # test finite state machine
    if modepygame == 0:
        tm.current_control()
        tm.set_cur_setpoint(0.0*A)
    elif modepygame == 1:
        tm.velocity_control()
        tm.set_vel_setpoint(0.0*rad/s)
    elif modepygame == 2:
        tm.current_control()
        error = nrj-(-nrj0+8*rad*rad/(s*s))
        torque=(-0.05*thetap*error).magnitude
        torque=maximum(minimum(torque,4.0),-4.0)
        # print("{:.2f}\t>{:.1f}\t>{:.1f}\t>{:.1f}".format(torque,nrj,theta,thetap))
        tm.set_cur_setpoint(torque)
        if cos(tm.encoder_estimates.position.to(rad)-theta0)<-0.95:
            modepygame = 3
    elif modepygame == 3:
        # control to the closest upright position
        tm.position_control()
        tm.set_pos_setpoint(round((tm.encoder_estimates.position.to(rad)-theta0)/(2*pi*rad)+0.5)*2*pi-pi+theta0)
        if cos(tm.encoder_estimates.position.to(rad)-theta0)>-0.95:
            modepygame = 2

    # display in window
    text1 = "Current : {:.2f}".format(tm.Iq.estimate)
    img1 = font.render(text1, True, pygame.color.THECOLORS['red'])
    rect1 = img1.get_rect()
    pygame.draw.rect(img1, pygame.color.THECOLORS['blue'], rect1, 1)

    text2 = "Position : {:.0f}".format(theta.to(deg))
    img2 = font.render(text2, True, pygame.color.THECOLORS['red'])
    rect2 = img2.get_rect()
    pygame.draw.rect(img2, pygame.color.THECOLORS['blue'], rect2, 1)

    text4 = "Velocity : {:.0f}".format(thetap.to(deg/s))
    img4 = font.render(text4, True, pygame.color.THECOLORS['red'])
    rect4 = img4.get_rect()
    pygame.draw.rect(img4, pygame.color.THECOLORS['blue'], rect4, 1)

    text5 = "Energy : {:.2f}".format(nrj)
    img5 = font.render(text5, True, pygame.color.THECOLORS['red'])
    rect5 = img5.get_rect()
    pygame.draw.rect(img5, pygame.color.THECOLORS['blue'], rect5, 1)

    text6 = "Temperature : {:.0f}".format(Q_(tm.device_info.temp, degC))
    img6 = font.render(text6, True, pygame.color.THECOLORS['red'])
    rect6 = img6.get_rect()
    pygame.draw.rect(img6, pygame.color.THECOLORS['blue'], rect6, 1)
    
    img100 = font.render("Error code : %.0f" % tm.state.error, True, pygame.color.THECOLORS['red'])
    rect100 = img100.get_rect()
    pygame.draw.rect(img100, pygame.color.THECOLORS['blue'], rect100, 1)

    img101 = font.render("State : %.0f, Mode : %.0f" % (tm.state.state,tm.state.mode), True, pygame.color.THECOLORS['red'])
    rect101 = img101.get_rect()
    pygame.draw.rect(img101, pygame.color.THECOLORS['blue'], rect101, 1)

    ecran.fill(pygame.color.THECOLORS['black'])
    ecran.blit(img1, (20, 20))
    ecran.blit(img2, (20, 60))
    ecran.blit(img4, (20, 100))
    ecran.blit(img5, (20, 140))
    ecran.blit(img100, (20, 180))
    ecran.blit(img101, (20, 220))
    pygame.display.update()

    # dirty sleep (no need of accuracy here)
    time.sleep(dt)

# stop calm procedure
print("ARRET")
tm.estop()

time.sleep(0.1)