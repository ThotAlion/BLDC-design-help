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

# geometry
# Knee
ratioKnee = abs((1145-(-573)))/180.0 #9.54
# Thigh
ratioThigh = abs((-827-(881)))/180.0 #9.48
# geometry of the leg
L1 = 0.18025*m
L2 = 0.16005*m
LMAX = L1+L2

# sampling period (s)
dt = 0.001
goon = True
step=1*deg
theta1 = 0*deg
theta2 = 0*deg
theta10 = 0*deg
theta20 = 0*deg
t0 = 0
Y0= 0.0*m
Z0= 0.0*m
alpha = 0.0*deg
D = 0.0*m
E = 0.0*m
F = 0.0*m
Yfoot = 0.0*m
Zfoot = (L1+L2)*m
modepygame = -1




# channel = guess_channel(bustype_hint='slcan')
channel='/dev/ttyS7'
can_bus = can.Bus(bustype='slcan',channel=channel,bitrate=1000000)
iface = CAN(can_bus)
tm1 = Tinymovr(node_id=2, iface=iface)
tm2 = Tinymovr(node_id=1, iface=iface)

maxspeed = 800*turn/min
tm1.set_limits(velocity=maxspeed*1.5, current=10.0*A)
tm2.set_limits(velocity=maxspeed, current=14.0*A)
tm1.set_gains(position=50.0, velocity=0.0001)
tm2.set_gains(position=50.0, velocity=0.0001)
tm1.set_integrator_gains(velocity=0.001)
tm2.set_integrator_gains(velocity=0.000)

# print(tm1.motor_info)
# print(tm2.motor_info)
# print(tm1.device_info)
# print(tm2.device_info)

while goon:
    # sensors
    # keyboard
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            goon = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                goon = False
            elif event.key == pygame.K_DOWN and modepygame == 0:
                print ("bas")
                modepygame = 3
            elif event.key == pygame.K_UP and modepygame == 0:
                print ("haut")
                modepygame = 2
            elif event.key == pygame.K_LEFT and modepygame == 0:
                print ("gauche")
                modepygame = 4
            elif event.key == pygame.K_RIGHT and modepygame == 0:
                print ("droite")
                modepygame = 5
            elif event.key == pygame.K_SPACE:
                print ("space")
                modepygame = -2
            elif event.key == pygame.K_z:
                print ("ZERO")
                theta10 = tm1.encoder_estimates.position
                theta20 = tm2.encoder_estimates.position
                modepygame = 0
            elif event.key == pygame.K_p and modepygame == 0:
                print ("POINT")
                modepygame = 1
            elif event.key == pygame.K_o and modepygame == 0:
                print ("HAUTBAS")
                modepygame = 6
            elif event.key == pygame.K_a:
                print ("Step+")
                step+=1.0*deg
                step=minimum(step,100*deg)
                print(step)
            elif event.key == pygame.K_q:
                print ("Step-")
                step-=1.0*deg
                step=maximum(step,-100*deg)
                print(step)
            elif event.key == pygame.K_s:
                print ("STAY")
                modepygame = 0
            elif event.key == pygame.K_b:
                print ("p2p")
                modepygame = 7
                t0 = time.time()
                Y0= Y
                Z0= Z

                
        elif event.type == pygame.KEYUP:
            if modepygame == -2:
                modepygame = -2
            else:
                modepygame = 0

    # motors
    theta1m = (-tm1.encoder_estimates.position.to(deg)+theta10)/ratioThigh
    theta2m = (tm2.encoder_estimates.position.to(deg)-theta20)/ratioKnee
    Y = L1*sin(theta1m)+L2*sin(theta1m+theta2m)
    Z = -L1*cos(theta1m)-L2*cos(theta1m+theta2m)
    # print("theta1m,theta2m:"+str(theta1m.to(deg))+','+str(theta2m.to(deg)))
    # print("Y,Z:"+str(Y)+','+str(Z))

    

    # Finite state machine
    if modepygame == -1:
        print("CALM DOWN, CALIBRATE FIRST!")
        activated = False
        p2pactivated = False
    elif modepygame == -2:
        activated = False
    elif modepygame == 0:
        activated = True
        p2pactivated = False
        alpha = 0.0*deg
        E=0.0*m
        F=-LMAX
        D=0.0*m
    elif modepygame == 1:
        activated = True
        p2pactivated = False
        alpha = 0.0*deg
        E=0.0*m
        F=-0.7*(L1+L2)
        D=0.0*m
    elif modepygame == 2:
        activated = True
        p2pactivated = False
        alpha = +90.0*deg
        E=0.04*m
        F=-0.7*(L1+L2)
        D=0.04*m
    elif modepygame == 3:
        activated = True
        p2pactivated = False
        alpha = -90.0*deg
        E=0.04*m
        F=-0.7*(L1+L2)
        D=0.04*m
    elif modepygame == 4:
        activated = True
        p2pactivated = False
        alpha = 0.0*deg
        E=0.04*m
        F=-0.7*(L1+L2)
        D=0.04*m
    elif modepygame == 5:
        activated = True
        p2pactivated = False
        alpha = 180.0*deg
        E=0.04*m
        F=-0.7*(L1+L2)
        D=0.04*m
    elif modepygame == 6:
        activated = True
        p2pactivated = False
        alpha += step
        E=0.00*m
        F=-0.7*(L1+L2)
        D=0.03*m
    elif modepygame == 7:
        activated = False
        p2pactivated = True
        Yobj=0.0
        Zobj= - 0.7*LMAX
        Duration = 5.0


    if p2pactivated:
        # Control
        bary = maximum(0,minimum((time.time()-t0)/Duration,1))
        print("bary: "+str(bary))
        Yfoot = Y0*(1-bary)+Yobj*bary
        Zfoot = Z0*(1-bary)+Zobj*bary
        
        # foot trajectory
        # Yfoot = E*cos(alpha)
        # Zfoot = F+D*sin(alpha)
        # print("YFoot,Zfoot: ("+str(Yfoot)+','+str(Zfoot)+")")

        # lenght and angle of leg
        L = sqrt(Yfoot**2+Zfoot**2)
        L = maximum(minimum(L,LMAX*0.95),LMAX*0.3)
        # print("L: "+str(L))
        theta = arcsin(Yfoot/L)
        theta = maximum(minimum(theta,45*deg),-45*deg)
        # print("L,theta: ("+str(L)+','+str(theta.to(deg))+")")

        # motor angles
        theta2 = arccos((L**2-L1**2-L2**2)/(2*L1*L2))
        # print("theta2: "+str(theta2.to(deg)))
        # mesured length
        # print("theta20: "+str(theta20.to(deg)))
        # print("tm2.position: "+str(tm2.encoder_estimates.position.to(deg)))
        # print("ratioKnee: "+str(ratioKnee))
        # print("theta2m: "+str(theta2m.to(deg)))
        Lm = sqrt(L1**2+L2**2+2*L1*L2*cos(theta2m))
        # print("Lm: "+str(Lm))

        # compensation
        thetaC = arccos((Lm**2+L1**2-L2**2)/(2*L1*Lm))
        # print("thetaC: "+str(thetaC.to(deg)))

        theta1=theta-thetaC
        # print("theta1: "+str(theta1.to(deg)))

        tm1.position_control()
        tm1.set_pos_setpoint(theta10-ratioThigh*theta1)
        tm2.position_control()
        tm2.set_pos_setpoint(theta20+ratioKnee*theta2)
    elif activated:
        # foot trajectory
        Yfoot = E*cos(alpha)
        Zfoot = F+D*sin(alpha)
        # print("YFoot,Zfoot: ("+str(Yfoot)+','+str(Zfoot)+")")

        # lenght and angle of leg
        L = sqrt(Yfoot**2+Zfoot**2)
        L = maximum(minimum(L,LMAX*0.95),LMAX*0.3)
        # print("L: "+str(L))
        theta = arcsin(Yfoot/L)
        theta = maximum(minimum(theta,45*deg),-45*deg)
        # print("L,theta: ("+str(L)+','+str(theta.to(deg))+")")

        # motor angles
        theta2 = arccos((L**2-L1**2-L2**2)/(2*L1*L2))
        # print("theta2: "+str(theta2.to(deg)))
        # mesured length
        # print("theta20: "+str(theta20.to(deg)))
        # print("tm2.position: "+str(tm2.encoder_estimates.position.to(deg)))
        # print("ratioKnee: "+str(ratioKnee))
        # print("theta2m: "+str(theta2m.to(deg)))
        Lm = sqrt(L1**2+L2**2+2*L1*L2*cos(theta2m))
        # print("Lm: "+str(Lm))

        # compensation
        thetaC = arccos((Lm**2+L1**2-L2**2)/(2*L1*Lm))
        # print("thetaC: "+str(thetaC.to(deg)))

        theta1=theta-thetaC
        # print("theta1: "+str(theta1.to(deg)))

        tm1.position_control()
        tm1.set_pos_setpoint(theta10-ratioThigh*theta1)
        tm2.position_control()
        tm2.set_pos_setpoint(theta20+ratioKnee*theta2)
    else:
        tm1.current_control()
        tm1.set_cur_setpoint(0.0*A)
        tm2.current_control()
        tm2.set_cur_setpoint(0.0*A)


    # display
    text1 = "Current :  {:.2f}||{:.2f}".format(tm1.Iq.estimate,tm2.Iq.estimate)
    img1 = font.render(text1, True, pygame.color.THECOLORS['red'])
    rect1 = img1.get_rect()
    pygame.draw.rect(img1, pygame.color.THECOLORS['blue'], rect1, 1)

    text2 = "Position : {:.0f}||{:.0f}".format(tm1.encoder_estimates.position.to(deg),tm2.encoder_estimates.position.to(deg))
    img2 = font.render(text2, True, pygame.color.THECOLORS['red'])
    rect2 = img2.get_rect()
    pygame.draw.rect(img2, pygame.color.THECOLORS['blue'], rect2, 1)

    text3 = "Controlled position 1 : {:.0f}||{:.0f}".format(theta1.to(deg),theta2.to(deg))
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