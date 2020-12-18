from numpy import *
import pygame
import time
import can
from tinymovr import Tinymovr
from tinymovr.iface.can import CAN, guess_channel
from tinymovr.units import get_registry
# Definition of units
ureg = get_registry()
Q_ = ureg.Quantity
A = ureg.ampere
s = ureg.second
min = ureg.minute
rad = ureg.radian
deg = ureg.degree
turn = ureg.turn
degC = ureg.degC


pygame.init()
ecran = pygame.display.set_mode((600, 600))
font = pygame.font.SysFont(None, 48)

dt = 0.01
step=1*deg
current_threshold = 8.0*A
sign = 0
ratio = 9
goon = True
position = 0*deg
modepygame = 0

# channel = guess_channel(bustype_hint='slcan')
channel='/dev/ttyS6'
can_bus = can.Bus(bustype='slcan',channel=channel,bitrate=1000000)
iface = CAN(can_bus)
tm = Tinymovr(node_id=1, iface=iface)

tm.set_limits(velocity=2000*turn/min, current=13.0*A)
tm.set_gains(position=100.0, velocity=0.0001)

print(tm.motor_info)
print(tm.device_info)

while goon:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            goon = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                goon = False
            elif event.key == pygame.K_DOWN:
                print ("bas")
            elif event.key == pygame.K_UP:
                modepygame = 3
            elif event.key == pygame.K_LEFT:
                modepygame = 1
            elif event.key == pygame.K_RIGHT:
                modepygame = 2
            elif event.key == pygame.K_z:
                modepygame = 0 
                print ("ZERO")
                position = tm.encoder_estimates.position
            elif event.key == pygame.K_c:
                modepygame = 0 
                print ("CALIBRATION")
                tm.set_state(state=0,mode=0)
                tm.calibrate()
            elif event.key == pygame.K_r:
                print ("RESET")
                tm.reset()
            elif event.key == pygame.K_SPACE:
                print ("EMERGENCY STOP")
                modepygame = 0 
            elif event.key == pygame.K_p:
                sign = 1
                modepygame = 4
                count=0
            elif event.key == pygame.K_a:
                step+=1*deg
                step=minimum(maximum(step,0),50)
                print(step)
            elif event.key == pygame.K_q:
                step-=1*deg
                step=minimum(maximum(step,0),50)
                print(step)
        elif event.type == pygame.KEYUP:
            modepygame = 0

    if modepygame == 0:
        tm.current_control()
        tm.set_cur_setpoint(0.0*A)
    elif modepygame == 1:
        position+=step
        tm.position_control()
        tm.set_pos_setpoint(position)
    elif modepygame == 2:
        position-=step
        tm.position_control()
        tm.set_pos_setpoint(position)
    elif modepygame == 3:
        tm.position_control()
        tm.set_pos_setpoint(position)
    elif modepygame == 4:
        position+=sign*step
        tm.position_control()
        tm.set_pos_setpoint(position)
        if sign*tm.Iq.estimate>=current_threshold:
            count+=1
        else:
            count=0
        if count>=10:
            position0 = tm.encoder_estimates.position
            position = position0
            modepygame = 5
            print("pop")
    elif modepygame == 5:
        position-=sign*step
        tm.position_control()
        tm.set_pos_setpoint(position)
        if(tm.encoder_estimates.position<position0-ratio*170*deg):
            modepygame = 6
        if abs(position-tm.encoder_estimates.position)>ratio*90*deg:
            position = tm.encoder_estimates.position
            modepygame = 0
    elif modepygame == 6:
        position+=sign*step
        tm.position_control()
        tm.set_pos_setpoint(position)
        if(tm.encoder_estimates.position>position0-ratio*5*deg):
            modepygame = 5
        if abs(position-tm.encoder_estimates.position)>ratio*90*deg:
            position = tm.encoder_estimates.position
            modepygame = 0
    # print(position)

    text1 = "Current : {:.2f}".format(tm.Iq.estimate)
    img1 = font.render(text1, True, pygame.color.THECOLORS['red'])
    rect1 = img1.get_rect()
    pygame.draw.rect(img1, pygame.color.THECOLORS['blue'], rect1, 1)

    text2 = "Position : {:.0f}".format(tm.encoder_estimates.position.to(deg))
    img2 = font.render(text2, True, pygame.color.THECOLORS['red'])
    rect2 = img2.get_rect()
    pygame.draw.rect(img2, pygame.color.THECOLORS['blue'], rect2, 1)

    text3 = "Controlled position : {:.0f}".format(position.to(deg))
    img3 = font.render(text3, True, pygame.color.THECOLORS['red'])
    rect3 = img3.get_rect()
    pygame.draw.rect(img3, pygame.color.THECOLORS['blue'], rect3, 1)

    text4 = "Velocity : {:.0f}".format(tm.encoder_estimates.velocity.to(deg/s))
    img4 = font.render(text4, True, pygame.color.THECOLORS['red'])
    rect4 = img4.get_rect()
    pygame.draw.rect(img4, pygame.color.THECOLORS['blue'], rect4, 1)

    text5 = "Temperature : {:.0f}".format(Q_(tm.device_info.temp, degC))
    img5 = font.render(text5, True, pygame.color.THECOLORS['red'])
    rect5 = img5.get_rect()
    pygame.draw.rect(img5, pygame.color.THECOLORS['blue'], rect5, 1)
    
    img100 = font.render("Error code : %.0f" % tm.state.error, True, pygame.color.THECOLORS['red'])
    rect100 = img100.get_rect()
    pygame.draw.rect(img100, pygame.color.THECOLORS['blue'], rect100, 1)

    img101 = font.render("State : %.0f, Mode : %.0f" % (tm.state.state,tm.state.mode), True, pygame.color.THECOLORS['red'])
    rect101 = img101.get_rect()
    pygame.draw.rect(img101, pygame.color.THECOLORS['blue'], rect101, 1)

    ecran.fill(pygame.color.THECOLORS['black'])
    ecran.blit(img1, (20, 20))
    ecran.blit(img2, (20, 60))
    ecran.blit(img3, (20, 100))
    ecran.blit(img4, (20, 140))
    ecran.blit(img5, (20, 180))
    ecran.blit(img100, (20, 220))
    ecran.blit(img101, (20, 260))
    pygame.display.update()
    
    time.sleep(dt)

print("ARRET")
tm.estop()

time.sleep(1)