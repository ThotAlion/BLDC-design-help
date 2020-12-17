from numpy import *
import pygame
import time
import can
from tinymovr import Tinymovr
from tinymovr.iface.can import CAN, guess_channel

pygame.init()
ecran = pygame.display.set_mode((600, 600))
font = pygame.font.SysFont(None, 48)

dt = 0.01
step=50
current_threshold = 8.0
sign = 1

goon = True
position = 0

position = 0
modepygame = 0

# channel = guess_channel(bustype_hint='slcan')
channel='/dev/ttyS7'
can_bus = can.Bus(bustype='slcan',
                           channel=channel,
                           bitrate=1000000)
iface = CAN(can_bus)
tm = Tinymovr(node_id=1, iface=iface)

tm.set_limits(velocity=200000.0, current=16.0)
tm.set_gains(position=50.0, velocity=0.0001)

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
        elif event.type == pygame.KEYUP:
            modepygame = 0

    if modepygame == 0:
        tm.current_control()
        tm.set_cur_setpoint(0.0)
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
        if sign == 1 and tm.Iq.estimate>=current_threshold:
            sign=-1
        if sign == -1 and tm.Iq.estimate<=-current_threshold:
            sign=1
    # print(position)

    text1 = "Current : {:.2f}".format(tm.Iq.estimate)
    img1 = font.render(text1, True, pygame.color.THECOLORS['red'])
    rect1 = img1.get_rect()
    pygame.draw.rect(img1, pygame.color.THECOLORS['blue'], rect1, 1)

    text2 = "Position : {:.0f}".format(tm.encoder_estimates.position)
    img2 = font.render(text2, True, pygame.color.THECOLORS['red'])
    rect2 = img2.get_rect()
    pygame.draw.rect(img2, pygame.color.THECOLORS['blue'], rect2, 1)


    text3 = "Controlled position : {:.0f}".format(position)
    img3 = font.render(text3, True, pygame.color.THECOLORS['red'])
    rect3 = img3.get_rect()
    pygame.draw.rect(img3, pygame.color.THECOLORS['blue'], rect3, 1)

    text4 = "Velocity : {:.0f}".format(tm.encoder_estimates.velocity)
    img4 = font.render(text4, True, pygame.color.THECOLORS['red'])
    rect4 = img4.get_rect()
    pygame.draw.rect(img4, pygame.color.THECOLORS['blue'], rect4, 1)
    
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
    ecran.blit(img100, (20, 180))
    ecran.blit(img101, (20, 220))
    pygame.display.update()
    
    time.sleep(dt)

print("ARRET")
tm.estop()

time.sleep(1)