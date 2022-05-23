#-------------------------------------------------------------------
# GUI: Animation of dynamic controlled 2 DOF RR planar manipulator 
# Libraries to install: Numpy, Pygame, math, Matplotlib
#-------------------------------------------------------------------
import numpy as np
from math import acos, atan2, pi, tau, sqrt, sin, cos, remainder
from nametoXY import WRD_XY
from arm_para import getLength, getmass, getgrv
import matplotlib.pyplot as plt
import pygame
from pygame.locals import *
import sys





er1_ar = [0]
er2_ar = [0]
time_ar = []
th1_ar = [0]
th2_ar = [0]

   
# ----------------- Controller variables -----------------------
er1 = 0
er2 = 0
pr_er1 = 0
pr_er2 = 0
erd1 = 0
erd2 = 0
pr_t = 0
tor1_ar=[]
#iterate through all desired angle once reached
n = 0 

#desired position in angle 
tdes1_ar = [0]
tdes2_ar = [0]
th1 = tdes1_ar[0]
th2 = tdes2_ar[0]

#Math: Energies, Dynamics & control + ODE solvers + Inverse kinematics + path planning
def energy(z):
    t1d = z[0]
    t2d = z[1]
    t1 = z[2]
    t2 = z[3]
    potential = m1*g*(-l1*cos(t1)+l1) + m2*g*(-l1*cos(t1)-l2*cos(t2)+l1+l2) 
    #- (m1*g*(-l1*cos(0)) + m2*g*(-l1*cos(0)-l2*cos(0)))
    kinetic = 0.5*m1*(l1*t1d)**2 + 0.5*m2*((t1d*l1)**2 + (t2d*l2)**2 + 2*t1d*t2d*l1*l2*cos(t1-t2))
    totenergy = potential + kinetic
    return potential, kinetic, totenergy
def f(z,t):
    t1d = z[0]
    t2d = z[1]
    t1 = z[2]
    t2 = z[3]

    global th1, th2
    global pr_t, pr_er1, pr_er2
    global er1_ar, er2_ar, time_ar
    global n, trace_color, tor1_ar
    global damp
    global start_time, stop_time, stop_flag
    

    #------------------------------------------Motion Equations-----------------------------------------------
    # Free  
    if a == 1:
        D1 = -t1d*damp
        D2 = -t2d*damp

        t1dd = -((D2*l1*cos(t1 - t2) + (l2*(-2*D1 + g*l1*(2*m1 + m2)*sin(t1) 
        + g*l1*m2*sin(t1 - 2*t2) + 2*l1*l2*m2*t2d**2* 
        + sin(t1 - t2) + l1**2*m2*t1d**2*sin(2*(t1 - t2))))/2)/(l1**2*l2*(m1 + m2 - m2*cos(t1 - t2)**2)))

        t2dd = -((D1*l2*m2*cos(t1 - t2) - (l1*(l2**2*m2**2*t2d**2*sin(2*(t1 - t2)) 
        + (m1 + m2)*(2*D2 + 2*l1*l2*m2*t1d**2*sin(t1 - t2) + g*l2*m2*sin(2*t1 - t2) 
        - g*l2*m2*sin(t2))))/2)/(l1*l2**2*m2*(m1 + m2 - m2*cos(t1 - t2)**2)))
    # Controlled  
    if a == 2 or a==3:
        ### ---------------------------------------  CONTROL SYSTEM PD controller  ----------------------------
        
        
        #proportional
        er1 = (th1 - t1)
        er2 = (th2 - t2)

        #Derivative
        dt = t - pr_t

        erd1 = -t1d
        erd2 = -t2d
        #control Torque link 1
        if abs(er1)>0.01:
            F1 = g*l1*(m1 + m2)*sin(t1) + kp*er1 + kd*erd1
        else: 
            F1 = g*l1*(m1 + m2)*sin(t1)

        
        #control Torque link 2
        if abs(er2)>0.01:
            F2 = m2*g*l2*sin(t2) + kp*er2 + kd*erd2
        else:
            F2 = m2*g*l2*sin(t2)
        
        #save previous error 
        pr_er1 = er1
        pr_er2 = er2
        
        pr_t = t
        er1_ar.append(er1)
        er2_ar.append(er2)
        time_ar.append(t)
        ### ----------------------------------- Path (error acceptance = 0.07) ------------------------------------------


        if abs(er1)<error_accept and abs(er2)<error_accept:
            if n==0:
                trace_color = LT_BLUE
            else:
                trace_color = BLUE
            if n<len(tdes1_ar):
                th1 = tdes1_ar[n]
                th2 = tdes2_ar[n]
                n=n+1
        
        
        tor1_ar.append(F1)

        t1dd = -((F2*l1*cos(t1 - t2) + (l2*(-2*F1 + g*l1*(2*m1 + m2)*sin(t1) 
        + g*l1*m2*sin(t1 - 2*t2) + 2*l1*l2*m2*t2d**2* 
        + sin(t1 - t2) + l1**2*m2*t1d**2*sin(2*(t1 - t2))))/2)/(l1**2*l2*(m1 + m2 - m2*cos(t1 - t2)**2)))

        t2dd = -((F1*l2*m2*cos(t1 - t2) - (l1*(l2**2*m2**2*t2d**2*sin(2*(t1 - t2)) 
        + (m1 + m2)*(2*F2 + 2*l1*l2*m2*t1d**2*sin(t1 - t2) + g*l2*m2*sin(2*t1 - t2) 
        - g*l2*m2*sin(t2))))/2)/(l1*l2**2*m2*(m1 + m2 - m2*cos(t1 - t2)**2)))
    
    
    
    th1_ar.append(th1)
    th2_ar.append(th2)

    zdot = np.array([t1dd, t2dd, t1d, t2d])
    return zdot
def RK4_method(y, t, dt):
    k1 = f(y,t)
    k2 = f(y+0.5*k1*dt, t+0.5*dt)
    k3 = f(y+0.5*k2*dt, t+0.5*dt)
    k4 = f(y+k3*dt, t+dt)
    avg = dt * (k1 + 2*k2 + 2*k3 + k4) /6
    return avg
def midpoint(y,t,dt):
    k1 = f(y,t)
    k2 = f(y+0.5*k1*dt, t+0.5*dt)
    return dt*k2
def euler(y,t,dt):
    k1 = f(y,t)
    return dt*k1
def pipi(input_angle):
    return remainder(input_angle, tau)
def ikin(pos):
    qdr = 0
    x2 = pos[0]
    y2 = pos[1]
    dist = sqrt(x2**2+y2**2)
    if dist< abs(l1-l2):
        print('postition out of workspace')
        x2 = x2*abs(l1-l2)/dist
        y2 = y2*abs(l1-l2)/dist
    if dist > (l1+l2):
        print('postition out of workspace')
        x2 = x2*(l1+l2)/dist
        y2 = y2*(l1+l2)/dist

    L3 = sqrt(x2**2+y2**2)
    t2 = acos((L3**2 - l1**2 - l2**2)/(2*l1*l2))
    t1 = atan2(y2*(l1 + l2*cos(t2))- x2*l2*sin(t2) , x2*(l1 + l2*cos(t2)) + y2*l2*sin(t2))
    t2 = t1 + t2 + pi/2
    t1 = t1+ pi/2
    return t1, t2
def XYtotheta(user_txt):
    global tdes1_ar, tdes2_ar,n
    x_ar , y_ar , word = WRD_XY(user_txt)
    current_t1 = y[2]
    current_t2 = y[3]
    #Reset all arrays
    tdes1_ar = []
    tdes2_ar = []
    n=0
    word_width = max(x_ar)-min(x_ar)
    circle_limit = 2*sqrt(3)-0.1
    if word_width > circle_limit:
        x_ar = x_ar*circle_limit/word_width

    for i in range(len(x_ar)):
        posit = [x_ar[i],y_ar[i]]
        tdes1,tdes2 = ikin(posit)
        tdes1 -= 2*pi
        tdes2 -= 2*pi
        tdes1_ar.append(tdes1)
        tdes2_ar.append(tdes2)
def target(th1,th2):
    global tdes1_ar, tdes2_ar ,n
    #Reset all arrays
    tdes1_ar = []
    tdes2_ar = []
    n=0

    tdes1_ar.append(th1)
    tdes2_ar.append(th2)

#Animation: Windows(mode selection) + robot(forward kin,model) + plots(angles,error,energy) + GUI options(pause, speed)
#GUI input: text, reset, waypoints(mouse position) + Game(colision detection rect, circle)  
def intro_get_mode():
    global y, tdes1_ar, tdes2_ar,l1,l2,m1,m2,g,damp, kp, kd, error_accept,n, trace_color, speed,th1,th2, offset,trace
    global drop_click, timer_count, game_running, game_success, game_over, got_obj
    loop = 1
    screen.fill(LT_BLUE)
    input_rect1 = pygame.Rect(w/2-120, h/2, 50, 32)
    input_rect2 = pygame.Rect(w/2-140, h/2+50, 50, 32)
    input_rect3 = pygame.Rect(w/2-140, h/2+100, 50, 32)
    offset = (w/5, h/2)
    drop_click = pygame.Rect(offset[0]-reach,offset[1]-reach, reach*2, reach*2)
    speed = 60
    while loop:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == K_ESCAPE:
                    sys.exit()
            if event.type == pygame.MOUSEBUTTONDOWN:
                if go_back.collidepoint(event.pos):
                    sys.exit()
                if input_rect1.collidepoint(event.pos):
                    mode = 1
                    trace_color = LT_BLUE
                    y = np.array([0, 0, 0, 0])
                    loop = 0
                if input_rect2.collidepoint(event.pos):
                    timer_count = 0.0
                    mode = 2
                    tdes1_ar = []
                    tdes2_ar = []
                    th1 = 0
                    th2 = 0
                    y = np.array([0, 0, 0, 0])
                    #desired position in angle 
                    screen.fill(WHITE)
                    trace = screen.copy()
                    l1,l2,m1,m2,g,damp, kp, kd, error_accept,n = reset_par()
                    loop = 0
                if input_rect3.collidepoint(event.pos):
                    mode = 3
                    timer_count = 0.0
                    got_obj = False
                    game_over = False
                    game_success = False
                    game_running = False
                    offset = (w/2,h/2)
                    drop_click = pygame.Rect(offset[0]-reach,offset[1]-reach, reach*2, reach*2)
                    tdes1_ar = []
                    tdes2_ar = []
                    th1 = 0
                    th2 = 0
                    y = np.array([0, 0, th1, th2])
                    #desired position in angle 
                    l1,l2,m1,m2,g,damp, kp, kd, error_accept,n = reset_par()
                    loop = 0
    
        pygame.draw.rect(screen, BLUE, input_rect1, 0, -1, 10,10,10,10)
        txt1 = 'Free'
        text_surface1 = textfontXL.render(txt1, True, WHITE)
        screen.blit(text_surface1, (input_rect1.x+20, input_rect1.y+5))
        input_rect1.w = max(100, text_surface1.get_width()+10) 

        pygame.draw.rect(screen, RED, input_rect2, 0, -1, 10,10,10,10)
        txt2 = 'Controlled'
        text_surface2 = textfontXL.render(txt2, True, WHITE)
        screen.blit(text_surface2, (input_rect2.x+5, input_rect2.y+5))
        input_rect2.w = max(100, text_surface2.get_width()+10) 

        pygame.draw.rect(screen, BLACK, input_rect3, 0, -1, 10,10,10,10)
        txt3 = 'Play game'
        text_surface3 = textfontXL.render(txt3, True, WHITE)
        screen.blit(text_surface3, (input_rect3.x+5, input_rect3.y+5))
        input_rect3.w = max(100, text_surface2.get_width()+10) 

        pygame.draw.rect(screen, BLACK, go_back, 0, -1, 15,15,15,15)
        text_surface1 = myfont.render("Exit", True, WHITE)
        screen.blit(text_surface1, (go_back.x+5, go_back.y+5))
        go_back.w = max(20, text_surface1.get_width()+10) 
        
        title = font_title.render('The Robotic Arm', True, BLACK)
        screen.blit(title, (w/2-300,h/2-200))

        sub_title = textfontXL.render('Choose a mode:', True, BLACK)
        screen.blit(sub_title, (w/2-170,h/2-50))

        madeby = textfont.render("Made by Idan Malka", True, BLACK)
        screen.blit(madeby, (w/2-140,h/2+200))
        madeby = textfont.render('3rd Year EEE Final Project', True, BLACK)
        screen.blit(madeby, (w/2-160,h/2+220))
        madeby = textfontL.render('University of Manchester', True, BLACK)
        screen.blit(madeby, (w/2-220,h/2+250))
        madeby = myfont.render('ID: 10534369', True, BLACK)
        screen.blit(madeby, (w/2-130,h/2+290))
        

        clock.tick(speed)
        pygame.display.update()

    screen.fill(WHITE)
    trace.fill(WHITE)
    return mode
def update(a1, a2):
    scale = 100
    x1 = l1*scale * sin(a1) + offset[0]
    y1 = l1*scale * cos(a1) + offset[1]
    x2 = x1 + l2*scale * sin(a2)
    y2 = y1 + l2*scale * cos(a2)

    return (x1, y1), (x2, y2)
def render(point1, point2):
    #arm visual
    scale = 5
    x1, y1,  = int(point1[0]), int(point1[1])
    x2, y2,  = int(point2[0]), int(point2[1])
    tip = x2,y2
    screen.blit(trace, (0,0))

    if prev_point:
        xp, yp = prev_point[0], prev_point[1]
        pygame.draw.line(trace, trace_color, (xp, yp), (x2, y2), 2)
 
    #waypoints visualization
    if wayx_arr:
        for i in range(len(wayx_arr)):
            pygame.draw.circle(screen, BLACK, (wayx_arr[i], wayy_arr[i]), 5, 2)

    pygame.draw.line(screen, BLACK, offset, (x1,y1), 5)
    pygame.draw.line(screen, BLACK, (x1,y1), (x2,y2), 5)
    pygame.draw.circle(screen, BLACK, offset, scale)
    pygame.draw.circle(screen, RED, (x1, y1), int(m1*scale+1))
    pygame.draw.circle(screen, BLUE, (x2, y2), int(m2*scale+1))

    return (x2, y2)
def render_game(point1, point2):
    global game_over, game_success, got_obj, game_running
    #arm visual
    scale = 5
    x1, y1,  = int(point1[0]), int(point1[1])
    x2, y2,  = int(point2[0]), int(point2[1])
    tip = x2,y2
    screen.blit(trace, (0,0))

    if prev_point:
        xp, yp = prev_point[0], prev_point[1]
        pygame.draw.line(trace, trace_color, (xp, yp), (x2, y2), 2)
 
    #waypoints visualization
    if wayx_arr:
        for i in range(len(wayx_arr)):
            pygame.draw.circle(screen, BLACK, (wayx_arr[i], wayy_arr[i]), 5, 2)

    if lvl == 1:
        #circle = centre, R
        text = textfontXL.render("Task Level 1", False, (0, 0, 0))
        screen.blit(text, (offset[0]-650, offset[1]-300))
        text = textfontL.render("Form a set of waypoints to", False, (0, 0, 0))
        screen.blit(text, (offset[0]-750, offset[1]+50))
        text = textfontL.render("Reach the green ball without collision", False, (0, 0, 0))
        screen.blit(text, (offset[0]-750, offset[1]+100))
        text = textfont.render("no time limit", False, (0, 0, 0))
        screen.blit(text, (offset[0]-750, offset[1]+150))
        pygame.draw.circle(screen, dRED, (circle[0], circle[1]), circle[2])
        pygame.draw.circle(screen, GREEN, (circle_tar[0], circle_tar[1]), circle_tar[2])
        for i in range(len(rect_lvl1)):
            pygame.draw.rect(screen, dRED, rect_lvl1[i])
            if rect_col(tip,rect_lvl1[i]) or circle_col(tip,circle):
                if game_success==False:
                    game_running=False
                    game_over=True
        
        if circle_col(tip,circle_tar):
            if game_over==False:
                game_running=False
                game_success = True

        pygame.draw.line(screen, BLACK, offset, (x1,y1), 5)
        pygame.draw.line(screen, BLACK, (x1,y1), (x2,y2), 5)
        pygame.draw.circle(screen, BLACK, offset, scale)
        pygame.draw.circle(screen, RED, (x1, y1), int(m1*scale+1))
        pygame.draw.circle(screen, BLUE, (x2, y2), int(m2*scale+1))
    elif lvl == 2:
        text = textfontXL.render("Task Level 2", False, (0, 0, 0))
        screen.blit(text, (offset[0]-700, offset[1]-300))
        text = textfontL.render("Grab the green ball without collision", False, (0, 0, 0))
        screen.blit(text, (offset[0]-750, offset[1]+50))
        if got_obj==False:
            pygame.draw.circle(screen, GREEN, (circle_tar[0], circle_tar[1]), circle_tar[2])
            
        pygame.draw.circle(screen, dRED, (circle[0], circle[1]), circle[2])
        pygame.draw.rect(screen, GREEN, box_tar,3)
        for i in range(len(rect_lvl2)):
            pygame.draw.rect(screen, dRED, rect_lvl2[i])
            if rect_col(tip,rect_lvl2[i]) or circle_col(tip,circle):
                if game_success==False:
                    game_over=True
                    game_running = False
        if circle_col(tip,circle_tar):
            got_obj= True
        if rect_col(tip,box_tar) and got_obj==True:
            if game_over==False:
                got_obj=False
                game_success = True
                game_running = False

        pygame.draw.line(screen, BLACK, offset, (x1,y1), 5)
        pygame.draw.line(screen, BLACK, (x1,y1), (x2,y2), 5)
        pygame.draw.circle(screen, BLACK, offset, scale)
        pygame.draw.circle(screen, RED, (x1, y1), int(m1*scale+1))
        pygame.draw.circle(screen, BLUE, (x2, y2), int(m2*scale+3),3)
        if got_obj==True:
            pygame.draw.circle(screen, GREEN, (x2, y2), int(m2*scale))
            text = textfontL.render("Good work! Place the ball in the green box", False, (0, 0, 0))
            screen.blit(text, (offset[0]-750, offset[1]+100))

    return (x2, y2)
def render_plt1(angle):
    scale = 60
    zoom = 1
    
    x1, y1, = int((angle[0]*zoom+pi)*scale), int(2*pi*scale-(angle[1]*zoom+pi)*scale)

    if prev_angle:
       x1p, y1p, = prev_angle[0], prev_angle[1]
       if abs(x1-x1p)>scale:
          pygame.draw.line(plt1_trace, WHITE, (x1p,y1p), (x1,y1))
       elif abs(y1-y1p)>scale:
          pygame.draw.line(plt1_trace, WHITE, (x1p,y1p), (x1,y1))
       else:
          pygame.draw.line(plt1_trace, RED, (x1p,y1p), (x1,y1))
    
    
    screen.blit(plt1_trace, plt1_offset)
    rectplt1 = pygame.Rect((plt1_offset[0]-5, plt1_offset[1]-5), (int(pi*120+10), int(pi*120+10)))
    pygame.draw.rect(screen, LT_BLUE, rectplt1, 2)
    pygame.draw.line(screen, BLACK, (plt1_offset[0],plt1_offset[1]+pi*scale), (plt1_offset[0]+pi*120,plt1_offset[1]+pi*scale),1)
    pygame.draw.line(screen, BLACK, (plt1_offset[0]+pi*60,plt1_offset[1]), (plt1_offset[0]+pi*60,plt1_offset[1]+pi*120),1)
    text = textfont.render("t2", False, (0, 0, 0))
    screen.blit(text, (plt1_offset[0]+pi*scale+5, plt1_offset[1]))
    text = textfont.render("t1", False, (0, 0, 0))
    screen.blit(text, (plt1_offset[0]+pi*120-10, plt1_offset[1]+pi*scale-20))

    text = textfontXL.render("Joint Angles (rad):", False, (0, 0, 0))
    screen.blit(text, (plt1_offset[0], plt1_offset[1]-50))
    return (x1, y1)   
def render_plt2(error):
    #error = [time, angle, desired angle]
    scale = 500/zoom
    x1, y1, y2 = int(error[0]*scale), int((error[1]+pi)*60), int((error[2]+pi)*60)
    if sw == False:
        col = RED
    else: col = BLUE

    if prev_error:
       x1p, y1p, y2p = prev_error[0], prev_error[1], prev_error[2]
       if abs(x1-x1p)>100 or abs(y1-y1p)>100:
          pygame.draw.line(plt2_trace, WHITE, (x1p,y1p), (x1,y1))
          pygame.draw.line(plt2_trace, WHITE, (x1p,y2p), (x1,y2))
       else:
          pygame.draw.line(plt2_trace, BLACK, (x1p,y1p), (x1,y1))
          pygame.draw.line(plt2_trace, col, (x1p,y2p), (x1,y2))

    
    screen.blit(plt2_trace, plt2_offset)
    pygame.draw.line(screen, BLACK, plt2_offset,(plt2_offset[0], plt2_offset[1]+2*pi*60),1)
    pygame.draw.line(screen, BLACK, (plt2_offset[0]-2, plt2_offset[1]),(plt2_offset[0]+5, plt2_offset[1]),1)
    pygame.draw.line(screen, BLACK, (plt2_offset[0]-2, plt2_offset[1]+pi*60),(plt2_offset[0]+500, plt2_offset[1]+pi*60),1)
    pygame.draw.line(screen, BLACK, (plt2_offset[0]-2, plt2_offset[1]+2*pi*60),(plt2_offset[0]+5, plt2_offset[1]+2*pi*60),1)

    text = textfontxs.render("-3.14", False, (0, 0, 0))
    screen.blit(text, (plt2_offset[0]+5, plt2_offset[1]-10))
    text = textfontxs.render("0", False, (0, 0, 0))
    screen.blit(text, (plt2_offset[0]+5, plt2_offset[1]+pi*60))
    text = textfontxs.render("+3.14", False, (0, 0, 0))
    screen.blit(text, (plt2_offset[0]+5, plt2_offset[1]+2*pi*60))

    return (x1, y1, y2)
def render_nrg(t, energy):
    pot, kin, tot = energy
    if tot < 31:
        scale = 12
    else:
        scale = 6
    size_tot = tot*scale
    lim = 400
    if size_tot>=400:
        size_tot=400

    size_pot = size_tot*pot/tot
    size_kin = size_tot*kin/tot
    tot_rec = pygame.Rect(nrg_off[0]-20,nrg_off[1]+lim-size_tot, 15, size_tot)
    pot_rec = pygame.Rect(nrg_off[0]-18,nrg_off[1]+lim-size_pot, 4, size_pot)
    kin_rec = pygame.Rect(nrg_off[0]-11,nrg_off[1]+lim-size_kin, 4, size_kin)
    screen.blit(nrg_trace, nrg_off)
    pygame.draw.rect(screen,SKY_BLUE, tot_rec, 20)
    pygame.draw.rect(screen,GREEN, pot_rec, 20)
    pygame.draw.rect(screen,RED, kin_rec, 20)
    
    #error = [time, angle, desired angle]
    scale1 = 500/zoom_nrg
    x1, y1, y2, y3= int(t*scale1), lim - size_pot, lim - size_kin, int(lim-size_tot)

    if prev_nrg:
       x1p, y1p, y2p, y3p= prev_nrg[0], prev_nrg[1],prev_nrg[2],prev_nrg[3]
       if abs(x1-x1p)>100 or abs(y1-y1p)>150:
          pygame.draw.line(nrg_trace, WHITE, (x1p,y1p), (x1,y1))
       else:
          pygame.draw.line(nrg_trace, GREEN, (x1p,y1p), (x1,y1))
          pygame.draw.line(nrg_trace, RED, (x1p,y2p), (x1,y2))
          pygame.draw.line(nrg_trace, BLUE, (x1p,y3p), (x1,y3))

    
    pygame.draw.line(screen, BLACK, nrg_off,(nrg_off[0], nrg_off[1]+400),1)
    pygame.draw.line(screen, BLACK, (nrg_off[0], nrg_off[1]+400),(nrg_off[0]+500, nrg_off[1]+400),1)
    if scale==12:
        text = textfont.render("30", False, (0, 0, 0))
        screen.blit(text, (nrg_off[0]+5, nrg_off[1]+40))
        text = textfont.render("15", False, (0, 0, 0))
        screen.blit(text, (nrg_off[0]+5, nrg_off[1]+220))
    elif scale==6 and size_tot<400:
        text = textfont.render("60", False, (0, 0, 0))
        screen.blit(text, (nrg_off[0]+5, nrg_off[1]+40))
        text = textfont.render("30", False, (0, 0, 0))
        screen.blit(text, (nrg_off[0]+5, nrg_off[1]+220))
    elif size_tot>=400:
        scaling_ax = '{}'.format(round(tot*36/40))
        scaling_ax2 = '{}'.format(round(tot*36/80))
        text = textfont.render(scaling_ax, False, (0, 0, 0))
        screen.blit(text, (nrg_off[0]+5, nrg_off[1]+40))
        text = textfont.render(scaling_ax2, False, (0, 0, 0))
        screen.blit(text, (nrg_off[0]+5, nrg_off[1]+220))


    pygame.draw.line(screen, BLACK, (nrg_off[0]-5, nrg_off[1]+40),(nrg_off[0]+5, nrg_off[1]+40),1)
    pygame.draw.line(screen, BLACK, (nrg_off[0]-5, nrg_off[1]+220),(nrg_off[0]+5, nrg_off[1]+220),1)
    pot_str = 'pot:{}'.format(round(pot,1))
    text = myfont.render(pot_str, False, (0, 0, 0))
    screen.blit(text, (nrg_off[0]+100,nrg_off[1]-20))

    kin_str = '+ kin:{}'.format(round(kin,1))
    text = myfont.render(kin_str, False, (0, 0, 0))
    screen.blit(text, (nrg_off[0]+200,nrg_off[1]-20))

    tot_str = 'tot:{} = '.format(round(tot,1))
    text = myfont.render(tot_str, False, (0, 0, 0))
    screen.blit(text, (nrg_off[0],nrg_off[1]-20))

    text = textfontL.render("Energies:", False, (0, 0, 0))
    screen.blit(text, (nrg_off[0]+100,nrg_off[1]-70))
    return (x1,y1,y2,y3)
def pause():
    loop = 1
    text = textfontL.render("PAUSED", False, (0, 0, 0))
    screen.blit(text, (20,80))
    while loop:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                loop = 0
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    loop = 0
                if event.key == pygame.K_SPACE:
                    screen.fill((0, 0, 0))
                    loop = 0
        pygame.display.update()
        # screen.fill((0, 0, 0))
        clock.tick(60)
def skip(spd):
    if spd < 360000:
        spd = spd*2
    return spd
def slow(spd):
    if spd > 30:
        spd = spd/2
    return spd
def get_text_input():
    global user_text
    user_text=''
    loop = 1
    text = textfont.render("Type your name and press Enter", False, (0, 0, 0))
    screen.blit(text, (word_offset[0]-50, word_offset[1]-20))
    
    while loop:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                loop = 0
            if event.type == pygame.MOUSEBUTTONDOWN:
                if in_word.collidepoint(event.pos):
                    word = user_text
                    loop = 0
            if event.type == pygame.KEYDOWN:
                if event.key == K_ESCAPE:
                    loop = 0
                if event.key == pygame.K_BACKSPACE:
                    user_text = user_text[:-1]
                if event.key == K_RETURN:
                    word = user_text
                    loop = 0
                else:
                    user_text += event.unicode
                    
        pygame.draw.rect(screen, color, in_word)
  
        text_surface = myfont.render(user_text, True, WHITE)
        screen.blit(text_surface, (in_word.x+5, in_word.y+5))
        in_word.w = max(100, text_surface.get_width()+10) 
        pygame.display.update()
        clock.tick(60)
    trace.fill(WHITE)
    XYtotheta(word)
    return word
def reset_par():
    global wayx_arr,wayy_arr
    wayx_arr = []
    wayy_arr = []
    l1,l2 = getLength()
    m1,m2 = getmass()
    g = getgrv()
    damp = 0.0
    kp = 10
    kd = 10
    error_accept = 0.03
    n=0
    return l1,l2,m1,m2,g,damp, kp, kd, error_accept,n
def mouse_pos_pend(clk):
    global wayx_arr, wayy_arr

    scale = 100
    x,y = clk
    wayx_arr.append(x)
    wayy_arr.append(y)

    x2 = (x-offset[0])/scale
    y2 = (-y+offset[1])/scale
    posit = [x2,y2]
    
    return ikin(posit)
def waypoints():
    global tdes1_ar, tdes2_ar,n,way_nbr,wayx_arr, wayy_arr,screen, game_running, start_time
    tdes1_ar = []
    tdes2_ar = []
    n=0
    nbr_points = 0
    wayx_arr = []
    wayy_arr = []
    loop = 1
    while loop:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                loop = 0
            if event.type == pygame.MOUSEBUTTONDOWN:
                if way_rect.collidepoint(event.pos):
                    loop = 0
                if drop_click.collidepoint(event.pos):
                    theta1, theta2 = mouse_pos_pend(pygame.mouse.get_pos())
                    tdes1_ar.append(theta1)
                    tdes2_ar.append(theta2)
                    nbr_points +=1
            if event.type == pygame.KEYDOWN:
                if event.key == K_ESCAPE:
                    loop = 0
                if event.key == K_RETURN:
                    loop = 0

        way_nbr = "waypoint={}".format(nbr_points)
        button(way_rect, way_nbr, RED, 5)

        if wayx_arr:
            for i in range(len(wayx_arr)):
                pygame.draw.circle(screen, BLACK, (wayx_arr[i], wayy_arr[i]), 5, 2)
        if game_running==False:
            start_time = t
            game_running=True
        pygame.display.update()
        clock.tick(60)

    trace.fill(WHITE)
    return
def rect_col(point,rect):
    #if point inside rect return True
    xr, yr, w, h = rect
    x,y = point
    if x>xr and x<(xr+w):
        if y>yr and y<(yr+h):
            return True
    return False
def circle_col(point,circle):

    #if point inside circle return True
    cx,cy,r = circle
    x,y = point
    d = sqrt((x-cx)**2+(y-cy)**2)
    if d<r:
        return True
    return False
def button(rect, text, color=(100,100,255), shape=10):
    pygame.draw.rect(screen, color, rect, 0, -1, shape, shape, shape, shape)
    text_surface1 = myfont.render(text, True, WHITE)
    screen.blit(text_surface1, (rect.x + 5, rect.y + 5))
    rect.w = max(20, text_surface1.get_width() + 10)
def text_var(text, variable, offset, rounding = 1):
    var = text.format(round(variable, rounding))
    text1 = textfont.render(var, False, (0, 0, 0))
    screen.blit(text1, offset)
l1,l2,m1,m2,g,damp, kp, kd, error_accept,n = reset_par()

w, h = 1550, 800
WHITE = (240,240,255)
BLACK = (0,0,0)
RED = (255,0,0)
dRED = (200,0,0)
BLUE = (0,0,255)
SKY_BLUE = (100,100,255)
GREEN = (0,200,0)
YELLOW = (0,255,255)
LT_BLUE = (200,200,255)

#offsets
offset = (w/5, h/2)
plt1_offset = (2*w/5, h/2-20)
plt2_offset = (3*w/5+100, h/2-20)
param_offset = (750,100)
word_offset = [250,100]
way_offset = [500,100]
nrg_off = [3*w/5+100 , h/2-30]
nrg_off2 = [50 , 200]

screen = pygame.display.set_mode((w,h))
pygame.display.set_caption('Final Project: Double pendulum')
screen.fill(WHITE)
trace_color = LT_BLUE
trace = screen.copy()
plt1_trace = screen.copy()
plt2_trace = screen.copy()
nrg_trace = screen.copy()
pygame.display.update()
clock = pygame.time.Clock()
time_counter = 0.0
stop_flag = False
zoom = 20
zoom_nrg = 10

#Set initial condition
prev_point = None
prev_angle = None
prev_error = None
prev_nrg = None
t = 0.0
start_time = 0.0
stop_time = 0.0
timer_count = 0.0
del_t = 0.02
pot,kin,tot = 0.0,0.0,0.0
t10 = tdes1_ar[0]
t1d0 = 0 #rad/s
t20 = tdes2_ar[0]
t2d0 = 0
y = np.array([0, 0, 0, 0])
J_ctrl = 1
speed = 60


#fonts
pygame.font.init()
myfont = pygame.font.SysFont('calibri', 24)
textfontxs = pygame.font.SysFont('verdana', 8)
textfont = pygame.font.SysFont('verdana', 15)
textfontL = pygame.font.SysFont('verdana', 25)
textfontXL = pygame.font.SysFont('calibri', 30)
font_title = pygame.font.SysFont('verdana', 60)


#-------- INPUT blocks
go_back = pygame.Rect(10,5, 30, 30)
ODE_method_rect = pygame.Rect(param_offset[0]+100,param_offset[1]+100, 30, 30)
ode = 0
# - Free : add/remove size l1 l2, add/remove size m1,m2, def reset 
param_button_color = SKY_BLUE
add_l1 = pygame.Rect(param_offset[0],param_offset[1], 30, 30)
sub_l1 = pygame.Rect(param_offset[0],param_offset[1]+40, 30, 30)
add_l2 = pygame.Rect(param_offset[0]+60,param_offset[1], 30, 30)
sub_l2 = pygame.Rect(param_offset[0]+60,param_offset[1]+40, 30, 30)

add_m1 = pygame.Rect(param_offset[0]+150,param_offset[1], 30, 30)
sub_m1 = pygame.Rect(param_offset[0]+150,param_offset[1]+40, 30, 30)
add_m2 = pygame.Rect(param_offset[0]+210,param_offset[1], 30, 30)
sub_m2 = pygame.Rect(param_offset[0]+210,param_offset[1]+40, 30, 30)

add_dp = pygame.Rect(param_offset[0]+300,param_offset[1], 30, 30)

sub_dp = pygame.Rect(param_offset[0]+300,param_offset[1]+40, 30, 30)

reach = int((l1+l2)*100)
drop_click = pygame.Rect(offset[0]-reach,offset[1]-reach, reach*2, reach*2)


reset_param = pygame.Rect(param_offset[0]+360,param_offset[1], 30, 30)
# ---------------  Controlled: create rectangle for name
user_text = 'Your Name?'
in_word = pygame.Rect(word_offset[0], word_offset[1], 140, 32)
way_nbr = 'waypoints'
way_rect = pygame.Rect(way_offset[0], way_offset[1], 140, 32)
wayx_arr = []
wayy_arr = []
color_active = pygame.Color('lightskyblue3')
color_passive = pygame.Color('chartreuse4')
color = color_passive

add_kp = pygame.Rect(param_offset[0],param_offset[1], 30, 30)
sub_kp = pygame.Rect(param_offset[0],param_offset[1]+40, 30, 30)
add_kd = pygame.Rect(param_offset[0]+60,param_offset[1], 30, 30)
sub_kd = pygame.Rect(param_offset[0]+60,param_offset[1]+40, 30, 30)
add_er = pygame.Rect(param_offset[0]+150,param_offset[1], 30, 30)
sub_er = pygame.Rect(param_offset[0]+150,param_offset[1]+40, 30, 30)

#game
game_running = False
game_over = False
game_success = False
retry_rect = pygame.Rect(w/2+500,h/2+200, 100, 100)
sucess_rect = pygame.Rect(w/2+500,h/2+200, 100, 100)
lvl_rect = pygame.Rect(w/2+400,h/2, 100, 50)
lvl = 1
#task level 1
circle = [offset[0]+100, offset[1]-100, 30]
rect1 = pygame.Rect(offset[0]+70, offset[1],100,10)
rect2 = pygame.Rect(offset[0]-25, offset[1]-210,30,150)
rect3 = pygame.Rect(offset[0]-250, offset[1]+10,280,100)
rect4 = pygame.Rect(offset[0]+100, offset[1]-100,5,100)
rect_lvl1 = [rect1,rect2,rect3,rect4]
#target circle:
circle_tar = [offset[0]-150, offset[1]-100, 5]
#task level 2 move object
rect1 = pygame.Rect(offset[0]+70, offset[1],100,10)
rect2 = pygame.Rect(offset[0]-25, offset[1]-210,30,150)
rect3 = pygame.Rect(offset[0]-250, offset[1]+10,280,100)
rect4 = pygame.Rect(offset[0]+100, offset[1]-100,5,100)
box_tar = pygame.Rect(offset[0]+100, offset[1]+50,5,5) 
move_x = 0
rect_lvl2 = [rect1,rect2,rect3,rect4]
got_obj = False

#switch between plots button
sw_plt = pygame.Rect(plt2_offset[0]+400,plt2_offset[1]-50, 140, 32)
sw = False




while 1:
    a = intro_get_mode()
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()
        if event.type == pygame.KEYDOWN:
            if event.key == K_ESCAPE:
                sys.exit()
    while a == 1:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == K_ESCAPE:
                    a = intro_get_mode()
                if event.key == K_c:
                    trace.fill(WHITE)
                    plt1_trace.fill(WHITE)
                    wayx_arr=[]
                    wayy_arr=[]
                if event.key == K_SPACE:
                    pause()
                if event.key == K_RIGHT:
                    y[J_ctrl]=y[J_ctrl]+2
                if event.key == K_LEFT:
                    y[J_ctrl]=y[J_ctrl]-2
                if event.key == K_DOWN:
                    J_ctrl = 1
                if event.key == K_UP:
                    J_ctrl = 0
                if event.key == K_l:
                    l1 = l1+0.1
                if event.key == K_s:
                    speed = skip(speed)
                if event.key == K_d:
                    speed = slow(speed)
            if event.type == pygame.MOUSEBUTTONDOWN:
                if go_back.collidepoint(event.pos):
                    a = intro_get_mode()
                if add_l1.collidepoint(event.pos):
                    if l1<2:
                        l1 += 0.1
                if sub_l1.collidepoint(event.pos):
                    if l1>0.1:
                        l1 -= 0.1
                if add_l2.collidepoint(event.pos):
                    if l2<2:
                        l2 += 0.1
                if sub_l2.collidepoint(event.pos):
                    if l2>0.1:
                        l2 -= 0.1
                if add_m1.collidepoint(event.pos):
                    if m1<=5:
                        m1+=0.2
                if sub_m1.collidepoint(event.pos):
                    if m1>0.4:
                        m1-=0.2
                if add_m2.collidepoint(event.pos):
                    if m2<=5:
                        m2+=0.2
                if sub_m2.collidepoint(event.pos):
                    if m2>0.4:
                        m2-=0.2
                if add_dp.collidepoint(event.pos):
                    if damp<2:
                        damp+=0.1
                if sub_dp.collidepoint(event.pos):
                    if damp>0.0:
                        damp-=0.1
                    if damp <= 0.0:
                        damp = 0.0
                if reset_param.collidepoint(event.pos):
                    l1,l2,m1,m2,g,damp, kp, kd, error_accept,n = reset_par()
                if drop_click.collidepoint(event.pos):
                    wayx_arr = []
                    wayy_arr = []
                    t1_drop, t2_drop = mouse_pos_pend(pygame.mouse.get_pos())
                    y = [0,0,t1_drop,t2_drop]
                if ODE_method_rect.collidepoint(event.pos):
                    if ode == 2:
                        ode = 0
                    else:
                        ode+=1

        screen.fill(LT_BLUE)
        
        
        point1, point2 = update(y[2], y[3])
        prev_point = render(point1, point2)
        angle = [pipi(y[2]), pipi(y[3])]
        prev_angle = render_plt1(angle)

        reach = int((l1+l2)*100)
        drop_click = pygame.Rect(offset[0]-reach,offset[1]-reach, reach*2, reach*2)
        pygame.draw.circle(screen, BLUE, (int(plt1_offset[0]+prev_angle[0]),int(plt1_offset[1]+prev_angle[1])), 3)
        
        #-------------- Buttons: arm param (l,m,dp, reset) , exit, timer
        button(add_l1, "+L1")
        button(sub_l1, "-L1")
        button(add_l2, "+L2")
        button(sub_l2, "-L2")

        button(add_m1, "+m1")
        button(sub_m1, "-m1")
        button(add_m2, "+m2")
        button(sub_m2, "-m2")

        button(add_dp, "+dp")
        button(sub_dp, "-dp")

        button(reset_param, "reset")

        button(go_back, "back", BLUE)

        if ode==0:
            text_surface1 = "ODE RK4"
        elif ode==1:
            text_surface1 = "ODE midpoint"
        elif ode==2:
            text_surface1 = "ODE Euler"
        button(ODE_method_rect, text_surface1, BLUE, 5)

        #workspace
        pygame.draw.circle(screen, RED, offset, int((l1 + l2) * 100), 2)
        pygame.draw.circle(screen, GREEN, offset, int(abs(l1 - l2) * 100), 2)

        #text
        text_var('Time: {} seconds', t, (10,40))
        text_var('L1={}', l1, (param_offset[0],param_offset[1]-20))
        text_var('L2={}', l2, (param_offset[0]+60,param_offset[1]-20))
        text_var('m1={}', m1, (param_offset[0]+150,param_offset[1]-20))
        text_var('m2={}', m2, (param_offset[0]+210,param_offset[1]-20))
        text_var('damp={}', damp, (param_offset[0]+300,param_offset[1]-20))

        text = textfontXL.render("Click anywhere in the circle", False, (0, 0, 0))
        screen.blit(text, (offset[0]-150, offset[1]+250))

        text = textfont.render("shortcuts: Pause (space), speed up/down(s,d), clear (c)", False, (0, 0, 0))
        screen.blit(text, (offset[0]-150, offset[1]+280))

        text = textfont.render("rotate joint clock/anticlockwise(arrow right/left)", False, (0, 0, 0))
        screen.blit(text, (offset[0]-150, offset[1]+300))
        text = textfont.render("control joint 1/ joint 2 (arrow up/down) ", False, (0, 0, 0))
        screen.blit(text, (offset[0]-150, offset[1]+320))
        

        
        #calculate next step
        t += del_t
        if ode==0:
            y = y + RK4_method(y, t, del_t)
        elif ode==1:
            y = y + midpoint(y, t, del_t)
        elif ode==2:
            y = y + euler(y, t, del_t)
        
        if (t-time_counter)>zoom_nrg:
            time_counter = t
            nrg_trace.fill(WHITE)
        
        prev_nrg = render_nrg((t-time_counter),energy(y))
        clock.tick(speed)
        pygame.display.update()

    while a == 2:
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
            if event.type == pygame.MOUSEBUTTONDOWN:
                if go_back.collidepoint(event.pos):
                    a = intro_get_mode()
                if in_word.collidepoint(event.pos):
                    color = color_active
                    wayx_arr=[]
                    wayy_arr=[]
                    name_path = get_text_input()
                    color = color_passive
                if way_rect.collidepoint(event.pos):
                    waypoints()
                if add_kp.collidepoint(event.pos):
                    if kp<30:
                        kp += 1
                    kp = kp
                if sub_kp.collidepoint(event.pos):
                    if kp>0:
                        kp -= 1
                    kp = kp
                if add_kd.collidepoint(event.pos):
                    if kd<30:
                        kd += 0.5
                    kd = kd
                if sub_kd.collidepoint(event.pos):
                    if kd>0:
                        kd -= 0.5
                    kd = kd
                if sw_plt.collidepoint(event.pos):
                    sw = not sw
                if sub_er.collidepoint(event.pos):
                    if error_accept<0.1:
                        error_accept+=0.01
                if add_er.collidepoint(event.pos):
                    if error_accept>0.01:
                        error_accept-=0.01
                if reset_param.collidepoint(event.pos):
                    l1,l2,m1,m2,g,damp, kp, kd, error_accept,n = reset_par()
                if drop_click.collidepoint(event.pos):
                    wayx_arr = []
                    wayy_arr = []
                    theta1, theta2 = mouse_pos_pend(pygame.mouse.get_pos())
                    target(theta1,theta2)
            if event.type == pygame.KEYDOWN:
                if event.key == K_ESCAPE:
                    a = intro_get_mode()
                if event.key == K_SPACE:
                    pause()
                if event.key == K_s:
                    speed = skip(speed)
                if event.key == K_d:
                    speed = slow(speed)
                if event.key == K_c:
                    trace.fill(WHITE)
                    plt1_trace.fill(WHITE)
                if event.key == K_p:
                    fig, ax1 = plt.subplots()
                    ax1.plot(time_ar,tor1_ar)
                    ax1.set(xlabel='time', ylabel='torque 1 (N)', title='torque actuator 1')
                    ax1.grid()
                    fig.savefig("test1.png")
                    plt.show()
                    tor1_ar = []
                    time_ar = []

            
        # it will set background color of screen
        screen.fill(WHITE)
           
        point1, point2 = update(y[2], y[3])
        prev_point = render(point1, point2)
        angle = [pipi(y[2]), pipi(y[3])]
        prev_angle = render_plt1(angle)

        #text
        
        if (t-time_counter)>zoom:
            time_counter = t
            plt2_trace.fill(WHITE)
  
        if sw == False:
            error = [t-time_counter, angle[0], pipi(th1_ar[-1])]
            text = textfontXL.render("Theta 1 VS Desired angle:", False, (0, 0, 0))
            txt_switch = 'Show t2'
        else:
            error = [t-time_counter, angle[1], pipi(th2_ar[-1])]
            text = textfontXL.render("Theta 2 VS Desired angle:", False, (0, 0, 0))
            txt_switch = 'Show t1'
        screen.blit(text, (plt2_offset[0], plt2_offset[1]-50))
        prev_error = render_plt2(error)

        text = textfontXL.render("Click anywhere in the circle", False, (0, 0, 0))
        screen.blit(text, (offset[0]-150, offset[1]+250))
        text = textfont.render("shortcuts: Pause (space), speed up/down(s,d), clear (c)", False, (0, 0, 0))
        screen.blit(text, (offset[0]-150, offset[1]+280))

        #Buttons: modify PID constants

        button(add_kp, "+kp")
        button(sub_kp, "-kp")
        button(add_kd, "+kd")
        button(sub_kd, "-kd")
        button(add_er, "+pr")
        button(sub_er, "-pr")
        button(reset_param, "reset")
        button(go_back, "back", BLUE)
        button(sw_plt, txt_switch, color, 5)

        button(in_word, user_text, color, 0)
        button(way_rect, way_nbr, color, 0)

        #text
        text_var('Time: {} seconds', t, (10,40))

        text = textfontXL.render("Write something!", False, (0, 0, 0))
        screen.blit(text, (word_offset[0]-50, word_offset[1]-40))
        
        if n==0:
            timer_count = 0.0
            stop_flag = False
        elif n==1:
            start_time = t

        elif n<len(tdes1_ar) and n>1:
            timer_count = t-start_time

        text_var('Time taken: {} s', timer_count, (way_offset[0], way_offset[1]+40))
        text_var('kp={}', kp, (param_offset[0],param_offset[1]-20))
        text_var('kd={}', kd, (param_offset[0]+60,param_offset[1]-20))
        text_var('precision={}%', (-500*error_accept+100), (param_offset[0]+150,param_offset[1]-20))
        
        pygame.draw.circle(screen, RED, offset, int((l1+l2)*100), 2)
        t += del_t
        y = y + RK4_method(y, t, del_t) 

        clock.tick(speed)
        pygame.display.update()

    while a == 3:
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                sys.exit()
            if event.type == pygame.MOUSEBUTTONDOWN:
                if go_back.collidepoint(event.pos):
                    a = intro_get_mode()
                if way_rect.collidepoint(event.pos):
                    waypoints()
                if add_kp.collidepoint(event.pos):
                    if kp<50:
                        kp += 1
                if sub_kp.collidepoint(event.pos):
                    if kp>0:
                        kp -= 1
                if add_kd.collidepoint(event.pos):
                    if kd<50:
                        kd += 0.5
                if sub_kd.collidepoint(event.pos):
                    if kd>0:
                        kd -= 0.5
                if sub_er.collidepoint(event.pos):
                    if error_accept<0.1:
                        error_accept+=0.01
                if add_er.collidepoint(event.pos):
                    if error_accept>0.01:
                        error_accept-=0.01
                if reset_param.collidepoint(event.pos):
                    l1,l2,m1,m2,g,damp, kp, kd, error_accept,n = reset_par()
                if retry_rect.collidepoint(event.pos) and game_over:
                    got_obj=False
                    tdes1_ar = []
                    tdes2_ar = []
                    th1 = 0
                    th2 = 0
                    y = np.array([0, 0, th1, th2])
                    #desired position in angle 
                    l1,l2,m1,m2,g,damp, kp, kd, error_accept,n = reset_par()
                    game_over=False
                if sucess_rect.collidepoint(event.pos) and game_success:
                    game_success = False
                    got_obj = False
                    a = intro_get_mode()
                if lvl_rect.collidepoint(event.pos):
                    if lvl==1:
                        lvl=2
                    else:
                        lvl=1

                    got_obj=False
                    game_over=False
                    game_success=False
                    game_running = False
                    tdes1_ar = []
                    tdes2_ar = []
                    th1 = 0
                    th2 = 0
                    y = np.array([0, 0, th1, th2])
                    #desired position in angle 
                    l1,l2,m1,m2,g,damp, kp, kd, error_accept,n = reset_par()
                    
            if event.type == pygame.KEYDOWN:
                if event.key == K_ESCAPE:
                    a = intro_get_mode()
                if event.key == K_SPACE:
                    pause()
                if event.key == K_s:
                    speed = skip(speed)
                if event.key == K_d:
                    speed = slow(speed)
                if event.key == K_c:
                    trace.fill(WHITE)
                    plt1_trace.fill(WHITE)

            
        # it will set background color of screen
        screen.fill(WHITE)

        point1, point2 = update(y[2], y[3])
        prev_point = render_game(point1, point2)

        #task level 1
        circle = [offset[0]+100, offset[1]-100, 30]
        rect1 = pygame.Rect(offset[0]+70, offset[1],100,10)
        rect2 = pygame.Rect(offset[0]-25, offset[1]-210,30,150)
        rect3 = pygame.Rect(offset[0]-250, offset[1]+10,280,100)
        rect4 = pygame.Rect(offset[0]+100, offset[1]-100,5,100)
        rect_lvl1 = [rect1,rect2,rect3,rect4]
        #target circle:
        circle_tar = [offset[0]-150, offset[1]-100, 5]
        #task level 2 move object
        rect1 = pygame.Rect(offset[0]+70, offset[1],140,10)
        rect2 = pygame.Rect(offset[0]-25, offset[1]-210,30,150)
        rect3 = pygame.Rect(offset[0]-250, offset[1]+10,280,100)
        rect4 = pygame.Rect(offset[0]+100, offset[1]-100,5,100)
        circle_tar = [offset[0]-150, offset[1]-100, 5]
        box_tar = pygame.Rect(offset[0]+40, offset[1]-160,25,25) 
        rect_lvl2 = [rect1,rect2,rect3,rect4]
        if move_x<=0:
            bottom=False
        if move_x>50:
            bottom=True
        
        if bottom:
            move_x-=0.2
        else:
            move_x+=0.2
        rect_lvl2[1] = pygame.Rect(offset[0]-25, offset[1]-210+move_x,30,150)

        pygame.draw.rect(screen, param_button_color, lvl_rect, 0, -1, 10,10,10,10)
        if lvl==1:
            text="Go to Task 2"
        else:
            text="Back to task 1..."
        text_surface1 = textfontXL.render(text, True, WHITE)
        screen.blit(text_surface1, (lvl_rect.x+5, lvl_rect.y+5))
        lvl_rect.w = max(20, text_surface1.get_width()+10)


        #buttons : modify PID constants

        button(add_kp, "+kp")
        button(sub_kp, "-kp")
        button(add_kd, "+kd")
        button(sub_kd, "-kd")
        button(add_er, "+pr")
        button(sub_er, "-pr")
        button(reset_param, "reset")
        button(go_back, "back", BLUE)

        button(way_rect, way_nbr, color, 0)

        time_string = 'Time: {} seconds'.format(round(t,1))
        text = myfont.render(time_string, False, (0, 0, 0))
        screen.blit(text, (10,40))

        if game_running:
            timer_count = t-start_time

        time_taken = 'Time taken: {} s'.format(round(timer_count, 1))
        text = textfont.render(time_taken, False, (0, 0, 0))
        screen.blit(text, (way_offset[0], way_offset[1]+40))
        


        constkp = 'kp={}'.format(round(kp,1))
        text = textfont.render(constkp, False, (0, 0, 0))
        screen.blit(text, (param_offset[0],param_offset[1]-20))

        constkd = 'kd={}'.format(round(kd,1))
        text = textfont.render(constkd, False, (0, 0, 0))
        screen.blit(text, (param_offset[0]+60,param_offset[1]-20))

        precision = 'precision={}%'.format(round(-500*error_accept+100,1))
        text = textfont.render(precision, False, (0, 0, 0))
        screen.blit(text, (param_offset[0]+150,param_offset[1]-20))
        
        pygame.draw.circle(screen, RED, offset, int((l1+l2)*100), 2)

        #game levels, succeed, game over 
        if game_over:
            text = font_title.render("Game Over:(", False, BLACK)
            screen.blit(text, (w/2-200,h/2+200))

            pygame.draw.rect(screen, BLACK, retry_rect, 0, -1, 10,10,10,10)
            text_surface1 = font_title.render("RETRY", True, RED)
            screen.blit(text_surface1, (retry_rect.x+5, retry_rect.y+5))
            retry_rect.w = max(20, text_surface1.get_width()+10) 
        if game_success:
            text = font_title.render("Task Succeeded :)", False, BLUE)
            screen.blit(text, (w/2-200,h/2+200))

            pygame.draw.rect(screen, BLACK, retry_rect, 0, -1, 10,10,10,10)
            text_surface1 = font_title.render("Quit", True, WHITE)
            screen.blit(text_surface1, (retry_rect.x+5, retry_rect.y+5))
            retry_rect.w = max(20, text_surface1.get_width()+10) 

        t += del_t
        y = y + RK4_method(y, t, del_t) 

        clock.tick(speed)
        pygame.display.update()
