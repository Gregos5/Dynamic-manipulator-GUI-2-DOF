from numpy import *



def WRD_XY(user_txt):
    #wrd = input("(only a and b) Your Name is: ")
    wrd = user_txt
    x=[]
    y=[]
    for element in wrd:
        if element=='a':
            ax,ay = getA()
            x = concatenate([x,ax])
            y = concatenate([y,ay])

        if element=='b':
            bx,by = getB()
            x = concatenate([x,bx])
            y = concatenate([y,by])

        if element=='c':
            cx,cy = getC()
            x = concatenate([x,cx])
            y = concatenate([y,cy])
        
        if element=='d':
            dx,dy = getD()
            x = concatenate([x,dx])
            y = concatenate([y,dy])
        
        if element=='e':
            ex,ey = getE()
            x = concatenate([x,ex])
            y = concatenate([y,ey])

        if element=='f':
            fx,fy = getF()
            x = concatenate([x,fx])
            y = concatenate([y,fy])

        if element=='g':
            gx,gy = getG()
            x = concatenate([x,gx])
            y = concatenate([y,gy])

        if element=='h':
            hx,hy = getH()
            x = concatenate([x,hx])
            y = concatenate([y,hy])

        if element=='i':
            ix,iy = getI()
            x = concatenate([x,ix])
            y = concatenate([y,iy])

        if element=='j':
            jx,jy = getJ()
            x = concatenate([x,jx])
            y = concatenate([y,jy])

        if element=='k':
            kx,ky = getK()
            x = concatenate([x,kx])
            y = concatenate([y,ky])
        
        if element=='l':
            lx,ly = getL()
            x = concatenate([x,lx])
            y = concatenate([y,ly])
        
        if element=='m':
            mx,my = getM()
            x = concatenate([x,mx])
            y = concatenate([y,my])
        
        if element=='n':
            nx,ny = getN()
            x = concatenate([x,nx])
            y = concatenate([y,ny])
        
        if element=='o':
            ox,oy = getO()
            x = concatenate([x,ox])
            y = concatenate([y,oy])
        
        if element=='p':
            px,py = getP()
            x = concatenate([x,px])
            y = concatenate([y,py])
        
        if element=='q':
            qx,qy = getQ()
            x = concatenate([x,qx])
            y = concatenate([y,qy])
        
        if element=='r':
            rx,ry = getR()
            x = concatenate([x,rx])
            y = concatenate([y,ry])
        
        if element=='s':
            sx,sy = getS()
            x = concatenate([x,sx])
            y = concatenate([y,sy])
        
        if element=='t':
            tx,ty = getT()
            x = concatenate([x,tx])
            y = concatenate([y,ty])
        
        if element=='u':
            ux,uy = getU()
            x = concatenate([x,ux])
            y = concatenate([y,uy])
        
        if element=='v':
            vx,vy = getV()
            x = concatenate([x,vx])
            y = concatenate([y,vy])
        
        if element=='w':
            wx,wy = getW()
            x = concatenate([x,wx])
            y = concatenate([y,wy])
        
        if element=='x':
            xx,xy = getX()
            x = concatenate([x,xx])
            y = concatenate([y,xy])
        
        if element=='y':
            yx,yy = getY()
            x = concatenate([x,yx])
            y = concatenate([y,yy])

        
        if element=='z':
            zx,zy = getZ()
            x = concatenate([x,zx])
            y = concatenate([y,zy])

        else:
            x=x
            y=y


        x = x-1.2
        

    x=true_divide(x,3)
    #place word in the midlle 
    a = min(x)
    x = x + abs(a)/2
    return x,y,wrd


def getA():
    Ax = [0,0.125,0.25,0.375,0.5,0.625,0.75,0.875,1,1.125,1.25,1.375,1.5,1.3,1.1,0.9,0.7,0.5,0.7,0.9,1.1,1.3,1.5,1.625,1.75,1.875,2]
    Ay = [0,0.125,0.25,0.375,0.5,0.625,0.75,0.875,1,0.875,0.75,0.625,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.375,0.25,0.125,0]
    Ax = true_divide(Ax,2)
    Ay = true_divide(Ay,2)+0.5
    return Ax, Ay

def getB():
    Bx = [-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.3,-0.15,0,0.15,0.2,0.25,0.25,0.22,0.15,0,-0.15,-0.3,-0.5,-0.3,-0.15,0,0.15,0.3,0.4,0.47,0.5,0.5,0.47,0.4,0.3,0.15,0,-0.15,-0.15,-0.3,-0.3,-0.5]
    By = [0,0.2,0.4,0.5,0.6,0.8,1,1,1,0.98,0.95,0.9,0.8,0.75,0.7,0.6,0.55,0.5,0.5,0.5,0.5,0.5,0.5,0.48,0.45,0.4,0.33,0.27,0.22,0.18,0.1,0.05,0.02,0,0,0,0,0,0]
    Bx = true_divide(Bx,1)+0.5
    By = true_divide(By,2)+0.5
    return Bx, By

def getC():
    x = [1,0.9,0.8,0.7,0.6,0.5,0.4,0.3,0.2,0.15,0.1,0.05,0,0,0,0.05,0.1,0.15,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1]
    y = [0.8,0.9,0.95,0.98,1,1,1,1,0.98,0.95,0.9,0.8,0.7,0.5,0.3,0.2,0.1,0.05,0.02,0,0,0,0,0.02,0.05,0.1,0.2]
    x = true_divide(x,1)
    y = true_divide(y,2)+0.5
    return x, y

def getD():
    x = [0,0,0,0,0,0,0,0.2,0.4,0.6,0.7,0.8,0.9,0.95,0.98,1,1,1,0.98,0.95,0.9,0.8,0.7,0.6,0.4,0.2,0]
    y = [0,0.2,0.4,0.5,0.6,0.8,1,1,1,1,0.98,0.95,0.9,0.8,0.7,0.6,0.5,0.4,0.3,0.2,0.1,0.05,0.02,0,0,0,0]
    x = true_divide(x,1)
    y = true_divide(y,2)+0.5
    return x, y

def getE():
    x = [0,0,0,0,0,0,0,0.2,0.4,0.5,0.6,0.8,1,0,0.2,0.4,0.5,0.6,0.8,1,0,0.2,0.4,0.5,0.6,0.8,1]
    y = [0,0.2,0.4,0.5,0.6,0.8,1,1,1,1,1,1,1,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0,0,0,0,0,0,0]
    x = true_divide(x,1)
    y = true_divide(y,2)+0.5
    return x, y

def getF():
    x = [0,0,0,0,0,0,0,0.2,0.4,0.5,0.6,0.8,1,0,0.2,0.4,0.5]
    y = [0,0.2,0.4,0.5,0.6,0.8,1,1,1,1,1,1,1,0.5,0.5,0.5,0.5]
    x = true_divide(x,1)
    y = true_divide(y,2)+0.5
    return x, y

def getG():
    x = [0.8,0.8,0.75,0.7,0.6,0.5,0.4,0.3,0.2,0.15,0.1,0.05,0,0,0,0.05,0.1,0.15,0.2,0.3,0.4,0.5,0.6,0.7,0.75,0.8,0.8,0.7,0.6,0.5,0.7,0.9,1]
    y = [0.6,0.75,0.9,0.95,0.98,1,1,1,0.98,0.95,0.9,0.8,0.7,0.5,0.3,0.2,0.1,0.05,0.02,0,0,0,0.4,0.05,0.15,0.25,0.4,0.4,0.4,0.4,0.4,0.4,0.4]
    x = true_divide(x,1)
    y = true_divide(y,2)+0.5
    return x, y

def getH():
    x = [0,0,0,0,0,0,0,0,0.2,0.4,0.5,0.6,0.8,1,1,1,1,1,1,1,1,1,1]
    y = [0,0.2,0.4,0.5,0.6,0.8,1,0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.7,0.8,1,0.8,0.6,0.4,0.3,0.2,0]
    x = true_divide(x,1)
    y = true_divide(y,2)+0.5
    return x, y

def getI():
    x = [0,0.2,0.4,0.6,0.8,1,0.8,0.6,0.5,0.5,0.5,0.5,0.5,0.5,0.3,0.1,0,0.2,0.4,0.6,0.8,1]
    y = [0,0,0,0,0,0,0,0,0,0.2,0.4,0.6,0.8,1,1,1,1,1,1,1,1,1]
    x = true_divide(x,1)
    y = true_divide(y,2)+0.5
    return x, y

def getJ():
    x = [0,0.2,0.4,0.5,0.5,0.5,0.5,0.47,0.45,0.4,0.3,0.25,0.2,0.1,0]
    y = [1,1,1,1,0.8,0.6,0.4,0.25,0.2,0.1,0,0,0,0.1,0.2]
    x = true_divide(x,1)
    y = true_divide(y,2)+0.5
    return x, y

def getK():
    x = [0,0,0,0,0,0,0,0,0.2,0.4,0.6,0.8,1,0.8,0.6,0.4,0.2,0,0.2,0.4,0.6,0.8,1]
    y = [0,0.2,0.4,0.5,0.6,0.8,1,0.5,0.6,0.7,0.8,0.9,1,0.9,0.8,0.7,0.6,0.5,0.4,0.3,0.2,0.1,0]
    x = true_divide(x,1)
    y = true_divide(y,2)+0.5
    return x, y

def getL():
    x = [0,0,0,0,0,0,0.2,0.4,0.6,0.8,1]
    y = [1,0.8,0.6,0.4,0.2,0,0,0,0,0,0]
    x = true_divide(x,1)
    y = true_divide(y,2)+0.5
    return x, y

def getM():
    x = [0,0,0,0,0,0,0.2,0.4,0.6,0.8,1,1.2,1.4,1.6,1.8,2,2,2,2,2,2]
    y = [0,0.2,0.4,0.6,0.8,1,0.9,0.8,0.7,0.6,0.5,0.6,0.7,0.8,0.9,1,0.8,0.6,0.4,0.2,0]
    x = true_divide(x,2)
    y = true_divide(y,2)+0.5
    return x, y

def getN():
    x = [0,0,0,0,0,0,0.2,0.4,0.6,0.8,1,1,1,1,1,1]
    y = [0,0.2,0.4,0.6,0.8,1,0.8,0.6,0.4,0.2,0,0.2,0.4,0.6,0.8,1]
    x = true_divide(x,1)
    y = true_divide(y,2)+0.5
    return x, y

def getO():
    x = [0.8,1,1,1,1,0.8,0.6,0.5,0.4,0.2,0,0,0,0,0.2,0.4,0.5,0.6]
    y = [0.05,0.2,0.4,0.6,0.8,0.95,1,1,1,0.95,0.8,0.6,0.4,0.2,0.05,0,0,0]
    x = true_divide(x,1)
    y = true_divide(y,2)+0.5
    return x, y

def getP():
    x = [-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.3,-0.15,0,0.15,0.2,0.25,0.25,0.22,0.15,0,-0.15,-0.3,-0.5]
    y = [0,0.2,0.4,0.5,0.6,0.8,1,1,1,0.98,0.95,0.9,0.8,0.75,0.7,0.6,0.55,0.5,0.5,0.5]
    x = true_divide(x,1)+0.5
    y = true_divide(y,2)+0.5
    return x, y

def getQ():
    x = [0.8,1,1,1,1,0.8,0.6,0.5,0.4,0.2,0,0,0,0,0.2,0.4,0.5,0.6,0.8,0.9,1]
    y = [0.05,0.2,0.4,0.6,0.8,0.95,1,1,1,0.95,0.8,0.6,0.4,0.2,0.05,0,0,0,0.3,0.15,0]
    x = true_divide(x,1)
    y = true_divide(y,2)+0.5
    return x, y

def getR():
    x = [-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.5,-0.3,-0.15,0,0.15,0.2,0.25,0.25,0.22,0.15,0,-0.15,-0.3,-0.5,-0.3,-0.15,-0.3,-0.15,0,0.15,0.3,0.4,0.5]
    y = [0,0.2,0.4,0.5,0.6,0.8,1,1,1,0.98,0.95,0.9,0.8,0.75,0.7,0.6,0.55,0.5,0.5,0.5,0.5,0.5,0.48,0.45,0.4,0.3,0.2,0.1,0]
    x = true_divide(x,1)+0.5
    y = true_divide(y,2)+0.5
    return x, y

def getS():
    x = [0,0.2,0.4,0.6,0.8,1,1,1,0.8,0.6,0.4,0.2,0,0,0,0.2,0.4,0.6,0.8,1]
    y = [0.2,0.05,0,0,0.05,0.2,0.3,0.4,0.45,0.5,0.5,0.5,0.6,0.7,0.8,0.95,1,1,0.95,0.8]
    x = true_divide(x,1)
    y = true_divide(y,2)+0.5
    return x, y

def getT():
    x = [0,0.2,0.4,0.6,0.8,1,0.8,0.6,0.5,0.5,0.5,0.5,0.5,0.5]
    y = [1,1,1,1,1,1,1,1,1,0.8,0.6,0.4,0.2,0]
    x = true_divide(x,1)
    y = true_divide(y,2)+0.5
    return x, y

def getU():
    x = [0,0,0,0,0.1,0.3,0.5,0.7,0.9,1,1,1,1]
    y = [1,0.8,0.6,0.4,0.2,0.05,0,0.05,0.2,0.4,0.6,0.8,1]
    x = true_divide(x,1)
    y = true_divide(y,2)+0.5
    return x, y

def getV():
    x = [0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1]
    y = [1,0.8,0.6,0.4,0.2,0,0.2,0.4,0.6,0.8,1]
    x = true_divide(x,1)
    y = true_divide(y,2)+0.5
    return x, y

def getW():
    x = [0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,1.1,1.2,1.3,1.4,1.5,1.6,1.7,1.8,1.9,2]
    y = [1,0.8,0.6,0.4,0.2,0,0.1,0.2,0.3,0.4,0.5,0.4,0.3,0.2,0.1,0,0.2,0.4,0.6,0.8,1]
    x = true_divide(x,2)
    y = true_divide(y,2)+0.5
    return x, y

def getX():
    x = [0,0.2,0.4,0.6,0.8,1,0.8,0.6,0.5,0.4,0.2,0,0.2,0.4,0.6,0.8,1]
    y = [0,0.2,0.4,0.6,0.8,1,0.8,0.6,0.5,0.6,0.8,1,0.8,0.6,0.4,0.2,0]
    x = true_divide(x,1)
    y = true_divide(y,2)+0.5
    return x, y

def getY():
    x = [0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1,0.8,0.6,0.5,0.5,0.5,0.5]
    y = [1,0.9,0.8,0.7,0.6,0.5,0.6,0.7,0.8,0.9,1,0.8,0.6,0.5,0.4,0.2,0]
    x = true_divide(x,1)
    y = true_divide(y,2)+0.5
    return x, y

def getZ():
    x = [0,0.2,0.4,0.6,0.8,1,0.8,0.6,0.4,0.2,0,0.2,0.4,0.6,0.8,1]
    y = [1,1,1,1,1,1,0.8,0.6,0.4,0.2,0,0,0,0,0,0]
    x = true_divide(x,1)
    y = true_divide(y,2)+0.5
    return x, y