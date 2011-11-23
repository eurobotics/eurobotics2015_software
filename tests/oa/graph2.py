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

def graph(filename, stx, sty, sta, enx, eny, opx, opy):
    cmd = "./main %d %d %d %d %d %d %d"%(stx, sty, sta, enx, eny, opx, opy)
    o,i = popen2.popen2(cmd)
    i.close()
    s = o.read(10000000)
    o.close()

    open(filename + ".txt", "w").write(s)

    if len(s) == 10000000:
        gloupix()

    fig = plt.figure()
    ax = fig.add_subplot(111)

    # area
    x,y = build_poly([(0,0), (3000,0), (3000,2100), (0,2100)])
    ax.plot(x, y, 'g-')
    
    x,y = build_poly([(945,495), (2055,495), (2055,1575), (945,1575)])
    ax.plot(x, y, 'g--')
     
    # black lines
    black_elements = [ Rectangle((400, 0), 50, 2100) ] 
    black_elements += [ Rectangle((2550, 0), 50, 2100) ] 
    
    # physical limits
    black_elements += [ Rectangle((0, 400), 400, 22) ]  
    black_elements += [ Rectangle((2600, 400), 400, 22) ]
    
    black_elements += [ Rectangle((450, 1850), 22, 250) ]
    black_elements += [ Rectangle((1128, 1850), 22, 250) ]
    
    black_elements += [ Rectangle((2528, 1850), 22, 250) ]
    black_elements += [ Rectangle((1850, 1850), 22, 250) ]
    
    black_elements += [ Rectangle((472, 1980), 656, 120) ]
    black_elements += [ Rectangle((1872, 1980), 656, 120) ]
 
    black_elements += [ Circle((975, 525), 50) ]
    black_elements += [ Circle((975, 1225), 50) ]

    black_elements += [ Circle((2025, 525), 50) ]
    black_elements += [ Circle((2025, 1225), 50) ]

    black_elements += [ Circle((1325, 1925), 50) ]
    black_elements += [ Circle((1675, 1925), 50) ]

    
    # green areas
    green_areas = [ Rectangle((0, 422), 400, 1678) ]
    green_areas += [ Rectangle((2600, 422), 400, 1678) ]
   
    # start areas
    blue_slots = [ Rectangle((0, 0), 400, 400) ]
    red_slots = [ Rectangle((2600, 0), 400, 400) ]     
      
    # slots
    color = 'blue'
    for i in range(0,6):
    
      if color == 'red':
         color = 'blue'
      else:
         color = 'red'
         
      for j in range(0,6):
         if color == 'red':
            color = 'blue'
            red_slots += [ Rectangle((450+(i*350), j*350), 350, 350) ]
         else:
            color = 'red'
            blue_slots += [ Rectangle((450+(i*350), j*350), 350, 350) ]
         
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

        m = re.match("oponent at: (-?\d+) (-?\d+)", l)
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
    

    p = PatchCollection(patches, cmap=matplotlib.cm.jet, alpha=0.4)
    ax.add_collection(p)
    
    p = PatchCollection(green_areas, cmap=matplotlib.cm.jet, alpha=0.4,  color='green')
    ax.add_collection(p)
    
    p = PatchCollection(red_slots, cmap=matplotlib.cm.jet, alpha=0.4,  color='red')
    ax.add_collection(p)

    p = PatchCollection(blue_slots, cmap=matplotlib.cm.jet, alpha=0.4,  color='blue')
    ax.add_collection(p)
    
    p = PatchCollection(black_elements, cmap=matplotlib.cm.jet, alpha=0.4,  color='black')
    ax.add_collection(p)


    x,y = build_path(path)
    ax.plot(x, y, 'bo-')

    ax.grid()
    ax.set_xlim(-100, 3100)
    ax.set_ylim(-100, 2500)
    #ax.set_title('spline paths')
    #plt.show()
    fig.savefig(filename)

# args are: startx, starty, starta, endx, endy, oppx, oppy

# paths to one slot from all others
random.seed(5)
for i in range(1,6):
   for j in range(1,5):
      x = 2
      y = 1
      name = "slot_%d%d_to_%d%d.png"%(x-1,y-1,i+1,j)
      startx = 625 + (x-1)*350
      starty = 175 + y*350
      endx = 625 + i*350
      endy = 175 + j*350
      #oppx = 625 + int(random.randint(2,5))*350
      #oppy = 175 + int(random.randint(2,4))*350
      oppx = -1000
      oppy = 0
      print (name, startx, starty, 0, endx, endy, oppx, oppy)
      graph(name, startx, starty, 0, endx, endy, oppx, oppy)

# in opponent
graph("in_opponent_00.png", 1500, 1050, 0, 2000, 1500, 1400, 900)
graph("in_opponent_01.png", 1500, 1050, 0, 1000, 500, 1400, 900)

#random.seed(0)
#for i in range(100):
#    stx = 150+450*int(random.randint(0,6))
#    sty = 722+250*int(random.randint(0,5))
#    enx = 150+450*int(random.randint(0,6))
#    eny = 722+250*int(random.randint(0,5))
#    opx = random.randint(-50, 3050)
#    opy = random.randint(-50, 2050)
#    name = "random%d.png"%(i)
#    print (name, stx, sty, 0, enx, eny, opx, opy)
#    graph(name, stx, sty, 0, enx, eny, opx, opy)
   
