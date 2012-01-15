import sys, re, math
import numpy as np
import matplotlib
import matplotlib.path as mpath
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
from matplotlib.patches import Arrow, Circle, Wedge, Polygon, Rectangle
from matplotlib.collections import PatchCollection
import popen2, random

Path = mpath.Path


def build_poly(ptlist):
    polydata = []
    polydata.append((Path.MOVETO, (ptlist[0])))
    for pt in ptlist[1:]:
        polydata.append((Path.LINETO, (pt)))
    polydata.append((Path.CLOSEPOLY, (ptlist[0])))
    codes, verts = zip(*polydata)
    poly = mpath.Path(verts, codes)
    x, y = zip(*poly.vertices)
    return x,y

def build_path(ptlist):
    polydata = []
    polydata.append((Path.MOVETO, (ptlist[0])))
    for pt in ptlist[1:]:
        polydata.append((Path.LINETO, (pt)))
    codes, verts = zip(*polydata)
    poly = mpath.Path(verts, codes)
    x, y = zip(*poly.vertices)
    return x,y

def draw_bottle(x, y):
    bottle = [ Rectangle((x-100, y), 200, 22*3) ]    
    bottle += [ Rectangle((x-11, y), 22, -22*2) ]
    return bottle   
    
def graph(filename, stx, sty, sta, enx, eny, op1x, op1y, op2x, op2y):
    cmd = "./main %d %d %d %d %d %d %d %d %d"%(stx, sty, sta, enx, eny, op1x, op1y, op2x, op2y)
    o,i = popen2.popen2(cmd)
    i.close()
    s = o.read(100000000)
    o.close()

    open(filename + ".txt", "w").write(s)

    if len(s) == 100000000:
        gloupix()

    fig = plt.figure()
    ax = fig.add_subplot(111)

    # total area limits
    x,y = build_poly([(0,0), (3000,0), (3000,2000), (0,2000)])
    ax.plot(x, y, 'k-')
    
    # play area limits
    x,y = build_poly([(400+260,260), (3000-400-260,260), (3000-400-260,2000-260-44), (400+260,2000-260-44)])
    ax.plot(x, y, 'c--')
     
    # sea
    sea = [ Rectangle((0,0), 3000, 2000) ]
        
    # black lines
    lines = [ Rectangle((500,450), 150, 20) ]
    lines += [ Rectangle((3000-500,450), -150, 20) ]
    lines += [ Rectangle((500+150-20,450+20), 20, 2000-(450+20)) ]
    lines += [ Rectangle((3000-(500+150-20),450+20), -20, 2000-(450+20)) ]
        
    # bottles
    red_bottles = draw_bottle(640, 2000)
    red_bottles += draw_bottle(3000-640-477, 2000)
    purple_bottles = draw_bottle(640+477, 2000)
    purple_bottles += draw_bottle(3000-640, 2000)
        
    # isles
    isles = [Circle((1100, 1000), 200)]   
    isles += [Circle((1100+800, 1000), 200)]
    isles += [Circle((1500, 1000), 150/2)]
    isles += [Wedge((1500, 0), 300, 0, 180)]
    
    # isles patch
    isles_patch = [Wedge((1500, 1000-750), 550, 45, 135)]
    isles_patch += [Wedge((1500, 1000+750), 550, 180+45, 180+135)]
     
    # beaches
    beaches = [Circle((1100, 1000), 300)]   
    beaches += [Circle((1100+800, 1000), 300)]
    beaches += [Wedge((1500, 0), 400, 0, 180)]
    beaches_patch = [Rectangle((1100, 1000-264.7), 800, 264.7*2)]
       
    # ships
    ships = [ Polygon([(0,500), (400,500), (350,2000), (0,2000)]) ]
    ships += [ Polygon([(3000-400,500), (3000,500), (3000,2000), (3000-350, 2000)]) ]

    # barrier
    barriers = [ Rectangle((0, 500), 400, 18, ls = 'solid') ]
    barriers += [ Rectangle((3000, 500), -400, 18, ls = 'solid') ]

    # holds
    holds = [ Rectangle((0, 2000), 340, -610, ls='solid') ]
    holds += [ Rectangle((3000, 2000), -340, -610,ls='solid') ]
        
    # totems
    totems = [ Rectangle((1100-125, 1000-125), 250, 250) ]
    totems += [ Rectangle((1100+800-125, 1000-125), 250, 250) ]
   
    # start areas (captain bedroom)
    start_area_red = [ Rectangle((0, 0), 500, 500) ]
    start_area_purple = [ Rectangle((2500, 0), 500, 500) ]     
     
         
    poly = None
    poly_wait_pts = 0
    start = None
    path = None
    patches = []
    for l in s.split("\n"):
        m = re.match("robot at: (-?\d+) (-?\d+) (-?\d+)", l)
        if m:
            x,y,a = (int(m.groups()[0]), int(m.groups()[1]), int(m.groups()[2]))
            path = [ (x,y) ]
            a_rad = (a * math.pi / 180.)
            dx = 150 * math.cos(a_rad)
            dy = 150 * math.sin(a_rad)
            patches += [ Circle((x, y), 50) ]
            patches += [ Arrow(x, y, dx, dy, 50) ]

        m = re.match("oa_start_end_points\(\) \((-?\d+),(-?\d+)\) \((-?\d+),(-?\d+)\)", l)
        if m:
            dst_x,dst_y = (int(m.groups()[2]), int(m.groups()[3]))
            patches += [ Circle((dst_x, dst_y), 50) ]

        m = re.match("oa_new_poly\(size=(-?\d+)\)", l)
        if m:
            poly_wait_pts = int(m.groups()[0])
            poly = []

        m = re.match("oponent 1 at: (-?\d+) (-?\d+)", l)
        if m:
            poly_wait_pts = 4
            poly = []

        m = re.match("oponent 2 at: (-?\d+) (-?\d+)", l)
        if m:
            poly_wait_pts = 4
            poly = []

        m = re.match("oa_poly_set_point\(\) \((-?\d+),(-?\d+)\)", l)
        if m:
            poly.append((int(m.groups()[0]), int(m.groups()[1])))
            poly_wait_pts -= 1
            if poly_wait_pts == 0:
                x,y = build_poly(poly)
                ax.plot(x, y, 'r-')

        m = re.match("GOTO (-?\d+),(-?\d+)", l)
        if m:
            path.append((int(m.groups()[0]), int(m.groups()[1])))

        m = re.match("With avoidance (-?\d+): x=(-?\d+) y=(-?\d+)", l)
        if m:
            path.append((int(m.groups()[1]), int(m.groups()[2])))
        
        m = re.match("nb_rays = (-?\d+)", l)
        if m:
            print((int(m.groups()[0])))
    

    
    p = PatchCollection(sea, cmap=matplotlib.cm.jet, alpha=1,  color='lightblue')
    ax.add_collection(p)
    
    p = PatchCollection(ships, cmap=matplotlib.cm.jet, alpha=1,  color='brown')
    ax.add_collection(p)
       
    p = PatchCollection(holds, cmap=matplotlib.cm.jet, alpha=0.5,  color='grey')
    ax.add_collection(p)

    p = PatchCollection(lines, cmap=matplotlib.cm.jet, alpha=1,  color='black')
    ax.add_collection(p)

    p = PatchCollection(beaches_patch, cmap=matplotlib.cm.jet, alpha=1,  color='yellow')
    ax.add_collection(p)

    p = PatchCollection(isles_patch, cmap=matplotlib.cm.jet, alpha=1,  color='lightblue')
    ax.add_collection(p)
    
    p = PatchCollection(beaches, cmap=matplotlib.cm.jet, alpha=1,  color='yellow')
    ax.add_collection(p)

    p = PatchCollection(isles, cmap=matplotlib.cm.jet, alpha=1,  color='green')
    ax.add_collection(p)

    p = PatchCollection(totems, cmap=matplotlib.cm.jet, alpha=1,  color='brown')
    ax.add_collection(p)
    
    p = PatchCollection(start_area_purple, cmap=matplotlib.cm.jet, alpha=1,  color='purple')
    ax.add_collection(p)

    p = PatchCollection(start_area_red, cmap=matplotlib.cm.jet, alpha=1,  color='red')
    ax.add_collection(p)   
    
    p = PatchCollection(red_bottles, cmap=matplotlib.cm.jet, alpha=1,  color='red')
    ax.add_collection(p)   
    
    p = PatchCollection(purple_bottles, cmap=matplotlib.cm.jet, alpha=1,  color='purple')
    ax.add_collection(p)   
   
    p = PatchCollection(patches, cmap=matplotlib.cm.jet, alpha=0.4)
    ax.add_collection(p)

    x,y = build_path(path)
    ax.plot(x, y, 'bo-')

    ax.grid()
    ax.set_xlim(-100, 3100)
    ax.set_ylim(-100, 2500)
    #ax.set_title('spline paths')
    #plt.show()
    fig.savefig(filename)

# args are: startx, starty, starta, endx, endy, opp1x, opp1y, opp2x, opp2y
#graph("single.png", 250, 250, 0, 2000, 1600, -1500, 0, -1500, 0)

# go in playground area
graph("go_in_area_1.png", 250, 250, 0, 2300, 1600, -1500, 0, -1500, 0)
graph("go_in_area_2.png", 250, 1000, 0, 2300, 1000, -1500, 0, -1500, 0)
graph("go_in_area_3.png", 250, 1000, 0, 2300, 260, -1500, 0, -1500, 0)
graph("go_in_area_4.png", 1000, 250, 0, 2300, 260, -1500, 0, -1500, 0)
graph("go_in_area_5.png", 1500, 240, 0, 680, 1600, -1500, 0, -1500, 0)
graph("go_in_area_6.png", 2300, 240, 0, 2000, 1400, -1500, 0, -1500, 0)

# op2_at_ship
graph("op2_at_ship_1.png", 1500, 350, 0, 1500, 1600, -1000, 0, 400, 700)
graph("op2_at_ship_2.png", 1500, 350, 0, 1500, 1600, -1000, 0, 400, 1000)
graph("op2_at_ship_3.png", 1500, 350, 0, 1500, 1600, -1000, 0, 400, 1200)
graph("op2_at_ship_4.png", 1500, 350, 0, 1500, 1600, -1000, 0, 500, 1600)


# random
#random.seed(5)
#for i in range(1,6):
#   for j in range(1,5):
#      x = 2
#      y = 1
#      name = "random_%d%d_to_%d%d.png"%(x-1,y-1,i+1,j)
#      startx = 625 + (x-1)*350
#      starty = 175 + y*350
#      endx = 625 + i*350
#      endy = 175 + j*350
#      #oppx = 625 + int(random.randint(2,5))*350
#      #oppy = 175 + int(random.randint(2,4))*350
#      oppx = -1000
#      oppy = 0
#      print (name, startx, starty, 0, endx, endy, oppx, oppy)
#      graph(name, startx, starty, 0, endx, endy, oppx, oppy)

# in opponent
#graph("in_opponent_00.png", 1500, 1050, 0, 2000, 1500, 1400, 900, 1400, 900)

#random.seed(0)
#for i in range(100):
#    stx = 680 + 250*int(random.randint(0,6))
#    sty = 300 + 250*int(random.randint(0,6))
#    enx = 680 + 250*int(random.randint(0,6))
#    eny = 300 + 250*int(random.randint(0,6))
#    op1x = 680 + 250*int(random.randint(0,6))
#    op1y = 300 + 250*int(random.randint(0,6))
#    op2x = 680 + 250*int(random.randint(0,6))
#    op2y = 300 + 250*int(random.randint(0,6))
#    name = "random%d.png"%(i)
#    print (name, stx, sty, 0, enx, eny, op1x, op1y, op2x, op2y)
#    graph(name, stx, sty, 0, enx, eny, op1x, op1y, op2x, op2y)
   
