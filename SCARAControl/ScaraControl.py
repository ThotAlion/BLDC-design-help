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
m = ureg.meter
turn = ureg.turn
degC = ureg.degC


pygame.init()
ecran = pygame.display.set_mode((800, 400))
font = pygame.font.SysFont(None, 48)

dt = 0.001
stepL=0.005*m
period=2*s
current_threshold = 8.0*A
sign = 0
ratio1 = 10.14
ratio2 = 9.5
goon = True
position1 = 0*deg
position2 = 0*deg
modepygame = 0

# geometry of the leg
L1 = 0.18025*m
L2 = 0.16005*m

# channel = guess_channel(bustype_hint='slcan')
channel='/dev/ttyS3'
can_bus = can.Bus(bustype='slcan',channel=channel,bitrate=1000000)
iface = CAN(can_bus)
tm1 = Tinymovr(node_id=1, iface=iface)
tm2 = Tinymovr(node_id=2, iface=iface)

tm1.set_limits(velocity=1400*turn/min, current=14.0*A)
tm2.set_limits(velocity=800*turn/min, current=18.0*A)
tm1.set_gains(position=50.0, velocity=0.0001)
tm2.set_gains(position=50.0, velocity=0.0001)
tm1.set_integrator_gains(velocity=0.001)
tm2.set_integrator_gains(velocity=0.001)

# print(tm1.motor_info)
# print(tm2.motor_info)
# print(tm1.device_info)
# print(tm2.device_info)

print ("ZERO")
position1 = tm1.encoder_estimates.position
position10 = position1
position2 = tm2.encoder_estimates.position
position20 = position2
length = L1+L2

while goon:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            goon = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                goon = False
            elif event.key == pygame.K_DOWN:
                print ("bas")
                modepygame = 2
            elif event.key == pygame.K_UP:
                print ("haut")
                modepygame = 1
            elif event.key == pygame.K_LEFT:
                print ("gauche")
            elif event.key == pygame.K_RIGHT:
                print ("droite")
            elif event.key == pygame.K_SPACE:
                print ("space")
                modepygame = 3
            elif event.key == pygame.K_z:
                modepygame = 0 
                print ("ZERO")
                position1 = tm1.encoder_estimates.position
                position10 = position1
                position2 = tm2.encoder_estimates.position
                position20 = position2
        elif event.type == pygame.KEYUP:
            modepygame = 0


    if modepygame == 0:
        tm1.current_control()
        tm1.set_cur_setpoint(0.0*A)
        tm2.current_control()
        tm2.set_cur_setpoint(0.0*A)
    elif modepygame == 1:
        length+=stepL
        if length<0.10*(L1+L2):
            length = 0.10*(L1+L2)
        if length>0.99*(L1+L2):
            length = 0.99*(L1+L2)
        position1 = ratio1*arccos((length**2-L1**2-L2**2)/(2*L1*L2))
        position2 = ratio2*arccos((L2**2-L1**2-length**2)/(-2*L1*length))
        # print(length)
        # print(L1)
        # print(L2)
        # print(position2)
        # print(position20)
        tm1.set_gains(position=100.0, velocity=0.0001)
        tm2.set_gains(position=100.0, velocity=0.0001)
        tm1.position_control()
        tm1.set_pos_setpoint(position10+position1)
        tm2.position_control()
        tm2.set_pos_setpoint(position20+position2)
    elif modepygame == 2:
        length-=stepL
        if length<0.10*(L1+L2):
            length = 0.10*(L1+L2)
        if length>0.99*(L1+L2):
            length = 0.99*(L1+L2)
        position1 = ratio1*arccos((length**2-L1**2-L2**2)/(2*L1*L2))
        position2 = ratio2*arccos((L2**2-L1**2-length**2)/(-2*L1*length))
        # print(length)
        # print(L1)
        # print(L2)
        # print(position2)
        # print(position20)
        tm1.set_gains(position=100.0, velocity=0.0001)
        tm2.set_gains(position=100.0, velocity=0.0001)
        tm1.position_control()
        tm1.set_pos_setpoint(position10+position1)
        tm2.position_control()
        tm2.set_pos_setpoint(position20+position2)
    elif modepygame == 3:
        
        position1 = ratio1*arccos((length**2-L1**2-L2**2)/(2*L1*L2))
        position2 = ratio2*arccos((L2**2-L1**2-length**2)/(-2*L1*length))
        tm1.set_gains(position=25.0, velocity=0.0001)
        tm2.set_gains(position=25.0, velocity=0.0001)
        tm1.position_control()
        tm1.set_pos_setpoint(position10+position1)
        tm2.position_control()
        tm2.set_pos_setpoint(position20+position2)

    text1 = "Current :  {:.2f}||{:.2f}".format(tm1.Iq.estimate,tm2.Iq.estimate)
    img1 = font.render(text1, True, pygame.color.THECOLORS['red'])
    rect1 = img1.get_rect()
    pygame.draw.rect(img1, pygame.color.THECOLORS['blue'], rect1, 1)

    text2 = "Position : {:.0f}||{:.0f}".format(tm1.encoder_estimates.position.to(deg),tm2.encoder_estimates.position.to(deg))
    img2 = font.render(text2, True, pygame.color.THECOLORS['red'])
    rect2 = img2.get_rect()
    pygame.draw.rect(img2, pygame.color.THECOLORS['blue'], rect2, 1)

    text3 = "Controlled position 1 : {:.0f}".format(position1.to(deg))
    img3 = font.render(text3, True, pygame.color.THECOLORS['red'])
    rect3 = img3.get_rect()
    pygame.draw.rect(img3, pygame.color.THECOLORS['blue'], rect3, 1)

    text4 = "Velocity : {:.0f}||{:.0f}".format(tm1.encoder_estimates.velocity.to(deg/s),tm2.encoder_estimates.velocity.to(deg/s))
    img4 = font.render(text4, True, pygame.color.THECOLORS['red'])
    rect4 = img4.get_rect()
    pygame.draw.rect(img4, pygame.color.THECOLORS['blue'], rect4, 1)

    text5 = "Temperature : {:.0f}||{:.0f}".format(Q_(tm1.device_info.temp, degC),Q_(tm2.device_info.temp, degC))
    img5 = font.render(text5, True, pygame.color.THECOLORS['red'])
    rect5 = img5.get_rect()
    pygame.draw.rect(img5, pygame.color.THECOLORS['blue'], rect5, 1)
    
    img100 = font.render("Error code : {}||{}".format(tm1.state.errors,tm2.state.errors), True, pygame.color.THECOLORS['red'])
    rect100 = img100.get_rect()
    pygame.draw.rect(img100, pygame.color.THECOLORS['blue'], rect100, 1)

    img101 = font.render("State : {}||{}, Mode : {}||{}".format(tm1.state.state,tm2.state.state,tm1.state.mode,tm2.state.mode), True, pygame.color.THECOLORS['red'])
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
tm1.estop()
tm2.estop()

time.sleep(1)