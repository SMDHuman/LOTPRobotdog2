from graphics import *
import math
import time
from PIL import Image
t=''' 
setup_image(resolution:(x,y) , scale , graph mode , background)
setup_screen(resolution:(x,y) , scale , graph mode)

save_image(file name)
update_screen(enable lines , enable dots)
clear_screen()

img_draw_dot((x,y),rgb color)

draw_dot((x,y))
draw_brush((x,y),radius)
draw_line((x1,y1) , (x2,y2))

mouse_position(pixel mode)
'''
def setup_image(res,scl,setmode,background='white'):
    global img,img_color,img_x,img_y,img_scale,img_setup
    img_setup=setmode
    img_scale=scl
    img_x,img_y=res[0],res[1]
    img_color=(0,0,0)
    img=Image.new('RGB',res,background)

def save_image(name):
    img.save(name)

def img_draw_dot(cor,clr):
    if(img_setup==4):
        img.putpixel((int(img_x/2)+round(cor[0]*img_scale),int(img_y/2)-round(cor[1]*img_scale)),clr)
    elif(img_setup==1):
        img.putpixel(cor[0]*img_scale,ys-cor[1]*img_scale,clr)
    elif(img_setup==2):
        img.putpixel(cor[0]*img_scale,int(img_y/2)-round(cor[1]*img_scale),clr)

def setup_screen(res,scl,setmode=4,lines='',dots='',up=0):
    global win,scale,xs,ys,setup,color
    if(dots=='dots'): dots=1
    else: dots=0
    if(lines=='lines'): lines=1
    else: lines=0
    if(not(up)):
        color='red'
        xs,ys=res[0],res[1]
        scale=scl
        win = GraphWin("Graph", xs, ys,autoflush=False)

    dotR=2
    if(setmode==4):
        setup=4
        if(lines):
            Line(Point(0,int(ys/2)),Point(xs,int(ys/2))).draw(win)
            Line(Point(int(xs/2),0),Point(int(xs/2),ys)).draw(win)
        if(dots):
            for y in range(round(ys/(scale*2))):
                for x in range(round(xs/(scale*2))):
                    color="red"
                    draw_brush((x,y),dotR)
                    draw_brush((-x,y),dotR)
                    draw_brush((x,-y),dotR)
                    draw_brush((-x,-y),dotR)
    elif(setmode==1):
        setup=1
        if(lines):
            Line(Point(0,0),Point(0,ys)).draw(win)
            Line(Point(xs,ys),Point(0,ys)).draw(win)
        if(dots):
            for y in range(round(ys/scale)):
                for x in range(round(xs/scale)):
                    color="red"
                    draw_brush((x,y),dotR)
    elif(setmode==2):
        setup=2
        if(lines):
            Line(Point(0,0),Point(0,ys)).draw(win)
            Line(Point(xs,int(ys/2)),Point(0,int(ys/2))).draw(win)
        if(dots):
            for y in range(round(ys/scale)):
                for x in range(round(xs/scale)):
                    color="red"
                    draw_brush((x,y),dotR)
                    draw_brush((x,-y),dotR)

def clear_screen():
    for item in win.items[:]:
        item.undraw()

def update_screen(lines='',dots=''):
    setup_screen(0,scale,setup,lines,dots,1)
    win.update()
            
def draw_dot(cor):
    if(setup==4):
        win.plot(int(xs/2)+round(cor[0]*scale),int(ys/2)-round(cor[1]*scale),color)
    elif(setup==2):
        win.plot(cor[0]*scale,ys-cor[1]*scale,color)
    elif(setup==1):
        win.plot(cor[0]*scale,int(ys/2)-round(cor[1]*scale),color)

def draw_brush(cor,r):
    if(setup==4):
        c=Circle(Point(int(xs/2)+round(cor[0]*scale),int(ys/2)-round(cor[1]*scale)),r)
        c.setFill(color)
        c.setOutline(color)
        c.draw(win)
    elif(setup==1):
        c=Circle(Point(cor[0]*scale,ys-cor[1]*scale),r)
        c.setFill(color)
        c.setOutline(color)
        c.draw(win)
    elif(setup==2):
        c=Circle(Point(cor[0]*scale,int(ys/2)-round(cor[1]*scale)),r)
        c.setFill(color)
        c.setOutline(color)
        c.draw(win)

def draw_line(cor1,cor2):
    if(setup==4):
        win.plot(int(xs/2)+round(cor1[0]*scale),int(ys/2)-round(cor1[1]*scale),color)
    elif(setup==1):
        l=Line(Point(cor1[0]*scale,ys-cor1[1]*scale),Point(cor2[0]*scale,ys-cor2[1]*scale))
        l.draw(win)
    elif(setup==2):
        l=Line(Point(cor1[0]*scale,int(ys/2)-round(cor1[1]*scale)),Point(cor2[0]*scale,int(ys/2)-round(cor2[1]*scale)))
        l.draw(win)

def mouse_position(pixelmode=0):
    win.bind('<Motion>', motion)
    if(pixelmode):
        return(pos)
    else:
        if(setup==4):
            return((pos[0]-xs/2)/scale,(ys/2-pos[1])/scale)
        elif(setup==1):
            return(pos[0]/scale,(ys-pos[1])/scale)
        else:
            return(pos[0]/scale,(ys/2-pos[1])/scale)

pos=(0,0)
def motion(event):
    global pos
    pos=(event.x, event.y)

def orbit():
    global color
    q=5
    orbit_speed=10
    orbit1_radius=5
    orbit2_radius=1
    loading=0
    for i in range(360*q):  
        #print(mouse_position(1))
        if(loading!=round(i*10/(36*q))):
            print("\n"*30,"%",loading)
            time.sleep(0.001)
            
        loading=round(i*10/(36*q))
        i/=q
        s=math.sin(math.radians(i))*orbit1_radius
        c=math.cos(math.radians(i))*orbit1_radius
        color='blue'
        img_draw_dot((c,s),(255,0,0))
        draw_dot((c,s))
        color='black'
        img_draw_dot((c+math.cos(math.radians(i*orbit_speed))*orbit2_radius,s+math.sin(math.radians(i*orbit_speed))*orbit2_radius),(0,255,0))
        draw_brush((c+math.cos(math.radians(i*orbit_speed))*orbit2_radius,s+math.sin(math.radians(i*orbit_speed))*orbit2_radius),2)

        if(win.checkMouse()):
            return()
    print("\n"*30)
    print("%",loading)
    win.getMouse()

if(__name__=="__main__"):
    setup_image((1000,1000),50,4)
    
    setup_screen((1000,700),50,4,0)

    orbit()
    save_image('orbit.png')
    win.close()