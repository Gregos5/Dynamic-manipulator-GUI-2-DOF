import math
import numpy

class Pendulum:
    def __init__(self):
        self.th1 = 0 # angle 1 in radians
        self.th2 = 0 # angle 2 in radians
        self.t1d = 0 # angle speed 1 rad/s
        self.t2d = 0 # angle speed 2 rad/s
        self.t1dd = 0 # angle accel 1 rad/s^2
        self.t2dd = 0 # angle accel 2 rad/s^2

        self.l1 = 1
        self.l2 = 1

        self.m1 = 1
        self.m2 = 1

        self.g = 9.81 # Gravity accel m/s^2

    def get_energy(self, z):
        t1d = z[0]
        t2d = z[1]
        t1 = z[2]
        t2 = z[3]
        potential = m1*g*(-l1*cos(t1)+l1) + m2*g*(-l1*cos(t1)-l2*cos(t2)+l1+l2) 
        #- (m1*g*(-l1*cos(0)) + m2*g*(-l1*cos(0)-l2*cos(0)))
        kinetic = 0.5*m1*(l1*t1d)**2 + 0.5*m2*((t1d*l1)**2 + (t2d*l2)**2 + 2*t1d*t2d*l1*l2*cos(t1-t2))
        totenergy = potential + kinetic
        return potential, kinetic, totenergy

    def update_state(self,z,t):
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