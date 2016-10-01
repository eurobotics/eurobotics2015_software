import math, sys, time, os, random, re
#from matplotlib.patches import Arrow, Circle, Wedge, Polygon, Rectangle
from visual import *

FLOAT = "([-+]?[0-9]*\.?[0-9]+)"
INT = "([-+]?[0-9][0-9]*)"

AREA_X = 3000.
AREA_Y = 2000.

ROBOT_HEIGHT = 350.0
WALL_HEIGHT = 70.0

#define ROBOT_LENGTH            288.5
#define ROBOT_WIDTH             330.0
#define ROBOT_CENTER_TO_FRONT   167.0
#define ROBOT_CENTER_TO_BACK    121.5
#define ROBOT_CENTER_TO_MOUTH	105.0

ROBOT_WIDTH  = 330.0
ROBOT_LENGTH = 288.5
#ROBOT_LENGTH = 210.0 #105.0*2.0
#ROBOT_X_OFFSET = ROBOT_LENGTH/2.0 - 121.5


#define ROBOT_LENGTH      	    163.
#define ROBOT_WIDTH 	    	210.
#define ROBOT_CENTER_TO_BACK    105.0
#define ROBOT_CENTER_TO_FRONT   (ROBOT_LENGTH-ROBOT_CENTER_TO_BACK)

ROBOT2_WIDTH  = 210.0
ROBOT2_LENGTH = 163.0
#ROBOT2_X_OFFSET = -ROBOT2_LENGTH/2.0 + (163-105)

area = [ (0.0, 0.0, -0.2), (3000.0, 2000.0, 0.2) ]
areasize = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]) , area)
area_box = box(size=areasize, color=(0.09, 0.38, 0.671))

scene.autoscale = 1

# all positions of robot every 5ms
save_pos = []
save_pos2 = []

robot = box(color=(1.0, 1.0, 1.0))
robot.opacity = 0.5

robot2 = box(color=(1.0, 1.0, 1.0))
robot2.opacity = 0.5

steam_shovel = box(color=(0.6, 0.6, 0.6))

lstick = box(color=(0.4, 0.4, 0.4))
rstick = box(color=(0.4, 0.4, 0.4))
arm = box(color=(0.06,0.3,0.54))
harvester = frame()

opp = box(color=(0.7, 0.2, 0.2))
opp.opacity = 0.7
opp2 = box(color=(0.2, 0.2, 0.7))
opp2.opacity = 0.7
last_pos = (0.,0.,0.)
last_pos2 = (0.,0.,0.)

hcenter_line = curve()
hcenter_line.pos = [(-AREA_X/2, 0., 0.3), (AREA_X/2, 0., 0.3)]
vcenter_line = curve()
vcenter_line.pos = [(0., -AREA_Y/2, 0.3), (0., AREA_Y/2, 0.3)]

wallx = [ (0.0, 0.0, -0.5), (AREA_X+44, 22, WALL_HEIGHT) ]
wallxsize = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]) , wallx)
wallx1_box = box(pos=(0,-AREA_Y/2-11, WALL_HEIGHT/2), size=wallxsize, color=(0.78, 0.09, 0.071))
wallx2_box = box(pos=(0,AREA_Y/2+11, WALL_HEIGHT/2), size=wallxsize, color=(0.78, 0.09, 0.071))

wally = [ (0.0, 0.0, -0.5), (22, AREA_Y+44, WALL_HEIGHT) ]
wallysize = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]) , wally)
wally1_box = box(pos=(-AREA_X/2-11, 0, WALL_HEIGHT/2), size=wallysize, color=(0.78, 0.09, 0.071))
wally2_box = box(pos=(AREA_X/2+11, 0, WALL_HEIGHT/2), size=wallysize, color=(0.78, 0.09, 0.071))


YELLOW = 0
GREEN    = 1
color = YELLOW

def square(sz):
    sq = curve()
    sq.pos = [(-sz, -sz, 0.3),
              (-sz, sz, 0.3),
              (sz, sz, 0.3),
              (sz, -sz, 0.3),
              (-sz, -sz, 0.3),]
    return sq

sq1 = square(250)
sq2 = square(500)

robot_x = 0.
robot_y = 0.
robot_a = 0.
robot2_x = 0.
robot2_y = 0.
robot2_a = 0.
robot_lstick_deployed = 0
robot_rstick_deployed = 0

lstick_deployed = 0
rstick_deployed = 0

lstick_offset = 0
rstick_offset = 0
arm_offset = -4
harvester_offset = 0

steam_shovel_offset = 0

robot_trail = curve()
robot_trail_list = []
robot2_trail = curve()
robot2_trail_list = []
max_trail = 500

area_objects = []

STEP_1_HEIGHT = 22.0
STEP_2_HEIGHT = 44.0
STEP_3_HEIGHT = 66.0
STEP_4_HEIGHT = 88.0
CLAP_HEIGHT= 30.0
CLAP_STICK_HEIGHT=150.0
STAND_HEIGHT=70.0
CUP_HEIGHT = 150

set_opp_nb = 1

def toggle_obj_disp():
    global area_objects
    if area_objects == []:

		set_home_areas()
		step_func(1,STEP_1_HEIGHT,YELLOW)
		step_func(2,STEP_2_HEIGHT,YELLOW)
		step_func(3,STEP_3_HEIGHT,YELLOW)
		step_func(4,STEP_4_HEIGHT,YELLOW)
		step_func(1,STEP_1_HEIGHT,GREEN)
		step_func(2,STEP_2_HEIGHT,GREEN)
		step_func(3,STEP_3_HEIGHT,GREEN)
		step_func(4,STEP_4_HEIGHT,GREEN)
		clap(-AREA_X/2+325,-AREA_Y/2-16,YELLOW,-80)
		clap(-AREA_X/2+620,-AREA_Y/2-16,GREEN,-80)
		clap(-AREA_X/2+920,-AREA_Y/2-16,YELLOW,-80)
		clap(+AREA_X/2-325,-AREA_Y/2-16,YELLOW,+80)
		clap(+AREA_X/2-620,-AREA_Y/2-16,GREEN,+80)
		clap(+AREA_X/2-920,-AREA_Y/2-16,YELLOW,+80)
		stand_yellow(-AREA_X/2+850,+AREA_Y/2-100)
		stand_yellow(-AREA_X/2+850,+AREA_Y/2-200)
		stand_yellow(-AREA_X/2+90,+AREA_Y/2-200)
		stand_green(+AREA_X/2-850,+AREA_Y/2-100)
		stand_green(+AREA_X/2-850,+AREA_Y/2-200)
		stand_green(+AREA_X/2-90,+AREA_Y/2-200)
		stand_green(+AREA_X/2-90,-AREA_Y/2+250)
		stand_green(+AREA_X/2-90,-AREA_Y/2+150)
		stand_yellow(-AREA_X/2+90,-AREA_Y/2+250)
		stand_yellow(-AREA_X/2+90,-AREA_Y/2+150)
		stand_yellow(-AREA_X/2+870,+AREA_Y/2-1355)
		stand_green(+AREA_X/2-870,+AREA_Y/2-1355)
		stand_yellow(-200,+AREA_Y/2-1400)
		stand_green(200,+AREA_Y/2-1400)
		stand_yellow(-400,-AREA_Y/2+230)
		stand_green(400,-AREA_Y/2+230)
		popcorn_func(-590,200)
		popcorn_func(590,200)
		popcorn_func(0,-650)
		popcorn_func(+AREA_X/2-250,-750)
		popcorn_func(-AREA_X/2+250,-750)
		cinema(-AREA_X/2+200,AREA_Y/2-400-189,GREEN)
		cinema(-AREA_X/2+200,-AREA_Y/2+400+189,GREEN)
		cinema(AREA_X/2-200,AREA_Y/2-400-189,YELLOW)
		cinema(AREA_X/2-200,-AREA_Y/2+400+189,YELLOW)
		popcorn_machine(-AREA_X/2+300,AREA_Y/2)
		popcorn_machine(-AREA_X/2+600,AREA_Y/2)
		popcorn_machine(+AREA_X/2-300,AREA_Y/2)
		popcorn_machine(+AREA_X/2-600,AREA_Y/2)
		platform()
		
    else:
        for o in area_objects:
			if o.visible:
				o.visible = 0
			else:
				o.visible = 1
				
def toggle_color():
    global color
    global GREEN, YELLOW
    if color == YELLOW:
        color = GREEN
    else:
        color = YELLOW
def popcorn_machine(x,y):
	global area_objects 
	base = [(0.0, 0.0,0.0),(48,48 ,170)]
	base_size = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]),base)
	c = box(pos=(x,y, 170/2), size=base_size,color =(0.09, 0.38, 0.671))
	area_objects.append(c)
	base_up = [(0.0, 0.0,0.0),(70,70 ,120)]
	base_up_size = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]),base_up)
	c = box(pos=(x,y,120/2+170+60), size=base_up_size,color =(0.09, 0.38, 0.671))
	c.opacity= 0.5
	area_objects.append(c)
	for i in range(5):
		c = sphere(pos=(x,y,170+25+i*50), radius=25)
		area_objects.append(c)
def set_home_areas():
	global area_objects 
	base = [(0.0, 0.0,0.0),(400,22 ,22)]
	base_size = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]),base)
	c = box(pos=(-AREA_X/2+200,-AREA_Y/2+1000-444/2,22), size=base_size,color =(0.988, 0.741, 0.2))
	area_objects.append(c)
	c = box(pos=(-AREA_X/2+200,-AREA_Y/2+1000+444/2,22), size=base_size,color =(0.988, 0.741, 0.2))
	area_objects.append(c)
	c = box(pos=(AREA_X/2-200,-AREA_Y/2+1000+444/2,22), size=base_size,color =(0.31, 0.7, 0.2))
	area_objects.append(c)
	c = box(pos=(AREA_X/2-200,-AREA_Y/2+1000-444/2,22), size=base_size,color =(0.31, 0.7, 0.2))
	area_objects.append(c)
	c=cylinder(axis=(0,0,1),length=0.5,radius=200, color=(0.31, 0.659, 0.2),pos=(AREA_X/2-450,-AREA_Y/2+1000, 0))
	area_objects.append(c)
	c=cylinder(axis=(0,0,1),length=0.5,radius=200, color=(0.988, 0.741, 0.122),pos=(-AREA_X/2+450,-AREA_Y/2+1000, 0))
	area_objects.append(c)
	base = [(0.0, 0.0,0.0),(22,444 ,22)]
	base_size = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]),base)
	c = box(pos=(-AREA_X/2+70,-AREA_Y/2+1000,22), size=base_size,color =(0.988, 0.741, 0.2))
	area_objects.append(c)
	base_size = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]),base)
	c = box(pos=(AREA_X/2-70,-AREA_Y/2+1000,22), size=base_size,color =(0.31, 0.7, 0.2))
	area_objects.append(c)
	c = sphere(pos=(AREA_X/2-70/2,-AREA_Y/2+1000,22+35), radius=35,color=(0,1,0))
	area_objects.append(c)
	c = sphere(pos=(-AREA_X/2+70/2,-AREA_Y/2+1000,22+35), radius=35,color=(0,1,0))
	area_objects.append(c)

	yellowarea = [ (0.0, 0.0, -0.5), (400.0,444.0 , 0.5) ]
	yellowareasize = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]) , yellowarea)
	c = box(pos=(-AREA_X/2+200,-AREA_Y/2+1000,0), size=yellowareasize, color=(0.988, 0.741, 0.122))
	area_objects.append(c)

	greenarea = [ (0.0, 0.0, -0.5), (400.0,444.0 , 0.5) ]
	greenareasize = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]) , greenarea)
	c = box(pos=(AREA_X/2-200,-AREA_Y/2+1000,0), size=greenareasize, color=(0.31, 0.659, 0.2))
	area_objects.append(c)

	
def platform():
	global area_objects 
	base = [(0.0, 0.0,0.0),(600,100 ,22)]
	base_size = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]),base)
	c = box(pos=(0,-AREA_Y/2+100/2, 0), size=base_size,color =(0.78, 0.09, 0.071))
	area_objects.append(c)

	green_area = [ (0.0, 0.0, -0.5), (800.0, 200.0, 0.5) ]
	green_area_size = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]) , green_area)
	c = box(pos=(0,-AREA_Y/2+200/2, 0), size=green_area_size, color=(0.9, 0.0, 0.0))
	area_objects.append(c)

	c = sphere(pos=(250,-AREA_Y/2+100/2,22+35), radius=35,color=(0,1,0))
	area_objects.append(c)
	c = sphere(pos=(-250,-AREA_Y/2+100/2,22+35), radius=35,color=(0,1,0))
	area_objects.append(c)

def clap(x,y,clap_color,offset):
	global area_objects 
	clap = [ (0.0, 0.0, -0.5), (160.0, 30.0, CLAP_HEIGHT) ]
	clapsize = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]) , clap)
	if clap_color == YELLOW:
		c = box(pos=(x,y, CLAP_HEIGHT/2+70), size=clapsize, color =(0.988, 0.741, 0.122))
	else:
		c = box(pos=(x,y, CLAP_HEIGHT/2+70), size=clapsize, color =(0.31, 0.659, 0.2))
	area_objects.append(c)

	clap = [ (0.0, 0.0, CLAP_HEIGHT), (20.0, 30.0, CLAP_STICK_HEIGHT) ]
	clapsize = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]) , clap)
	c = box(pos=(x+offset,y, CLAP_HEIGHT+CLAP_STICK_HEIGHT/2+70), size=clapsize, color =(0.5, 0.5, 0.5))
	area_objects.append(c)
def cinema(x,y,cinema_color):
	cinemaarea = [ (0.0, 0.0, -0.5), (400.0,378.0,0.5) ]
	cinemaareasize = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]) , cinemaarea)
	if cinema_color == YELLOW:
		c = box(pos=(x,y), size=cinemaareasize, color=(0.988, 0.741, 0.122))
	else:
		c = box(pos=(x,y), size=cinemaareasize, color=(0.31, 0.659, 0.2))
	area_objects.append(c)

def step_func(step_nr,height,step_color):
	global area_objects 
	step = [(0.0, 0.0, height-22.0),(533.0, 580.0-70*(step_nr-1),height)]
	step_size = reduce(lambda x,y:tuple([abs(x[i])+abs(y[i]) for i in range(len(x))]),step)
	if step_color == YELLOW:
		green_step=0
		c = box(pos=(-266.5+green_step,+AREA_Y/2-(580.0-70*(step_nr-1))/2,height/2), size=step_size,color =(0.988, 0.741, 0.122))
	else:
		green_step=266.5*2
		c = box(pos=(-266.5+green_step,+AREA_Y/2-(580.0-70*(step_nr-1))/2,height/2), size=step_size,color =(0.31, 0.659, 0.2))
	area_objects.append(c)

def stand_green(x,y):
	global area_objects
	c = cylinder(axis=(0,0,1),length=STAND_HEIGHT,radius=30, color=(0.31, 0.659, 0.2),pos=(x,y, 0))
	area_objects.append(c)

def stand_yellow(x,y):
	global area_objects
	c = cylinder(axis=(0,0,1),length=STAND_HEIGHT,radius=30, color=(0.988, 0.741, 0.122),pos=(x,y, 0))
	area_objects.append(c)
def popcorn_func(x,y):
	global area_objects
	c=cone(pos=(x,y, CUP_HEIGHT), axis=(0,0,-2*CUP_HEIGHT),radius=47.5)
	area_objects.append(c)
	#c = cylinder(axis=(0,0,1),length=CUP_HEIGHT,radius=27,pos=(x,y, 0))
	#area_objects.append(c)

def set_opp(x, y):
    opp.size = (300, 300, ROBOT_HEIGHT)
    opp.pos = (x, y, ROBOT_HEIGHT/2)

def set_opp2(x, y):
    opp2.size = (300, 300, ROBOT_HEIGHT)
    opp2.pos = (x, y, ROBOT_HEIGHT/2)

def set_lstick():
    global arm_offset
    global harvester_offset
    global rstick_offset, rstick_deployed
    global lstick_offset, lstick_deployed

    if lstick_offset == 3:
	lstick_offset = 0
	lstick_deployed = 0
    else:
    	lstick_offset = 3 
	lstick_deployed = 20
    	rstick_offset = 0
	rstick_deployed = 0
	arm_offset = -4
	harvester_offset = 0

def set_rstick():
    global arm_offset
    global harvester_offset
    global rstick_offset, rstick_deployed
    global lstick_offset, lstick_deployed
 
    if rstick_offset == 3:
	rstick_offset = 0
	rstick_deployed = 0
    else:
    	rstick_offset = 3
	rstick_deployed = 20
    	lstick_offset = 0
	lstick_deployed = 0
	arm_offset = -4
	harvester_offset = 0

def set_arm():
    global arm_offset
    global harvester_offset
    global rstick_offset, rstick_deployed
    global lstick_offset, lstick_deployed

    if arm_offset == 10:
	arm_offset = -4
    else:
    	arm_offset = 10
    	lstick_offset = 0
	lstick_deployed = 0
	rstick_deployed = 0
    	rstick_offset = 0
	harvester_offset = 0

def set_harvester():
    global arm_offset
    global rstick_offset, rstick_deployed
    global harvester_offset
    global lstick_offset, lstick_deployed

    if harvester_offset == -3:
	harvester_offset = 0
    else:
	arm_offset = -4
	lstick_deployed = 0
    	lstick_offset = 0
	rstick_deployed = 0
    	rstick_offset = 0
	harvester_offset = -3




def set_robot():
    global robot, last_pos, robot_trail, robot_trail_list
    global save_pos, robot_x, robot_y, robot_a
    global lstick_offset, lstick_deployed
    global rstick_offset, rstick_deployed
    global arm_offset, arm_deployed 

    if color == YELLOW:
        tmp_x = robot_x - AREA_X/2
        tmp_y = robot_y - AREA_Y/2
        tmp_a = robot_a
    else:
        tmp_x = -robot_x + AREA_X/2
        tmp_y = -robot_y + AREA_Y/2
        tmp_a = robot_a

	
    robot.pos = (tmp_x, tmp_y, ROBOT_HEIGHT/2)
    axis = (math.cos(tmp_a*math.pi/180),
            math.sin(tmp_a*math.pi/180),
            0)

    robot.axis = axis
    robot.size = (ROBOT_LENGTH, ROBOT_WIDTH, ROBOT_HEIGHT)
	

    """
	# Left stick
    lstick.pos = (tmp_x + ROBOT_X_OFFSET + (lstick_offset * 60 ) * math.cos((tmp_a-90)*math.pi/180),
                    tmp_y + (lstick_offset * 60) * math.sin((tmp_a-90)*math.pi/180),
                    ROBOT_HEIGHT/5)
    lstick.axis = axis
    lstick.size = (40,ROBOT_WIDTH-10+lstick_deployed, 20)
    lstick.color = (0.9, 0.2, 0.2)

	# Right stick
    rstick.pos = (tmp_x + ROBOT_X_OFFSET + (rstick_offset * 60 ) * math.cos((tmp_a+90)*math.pi/180),
                    tmp_y + (rstick_offset * 60) * math.sin((tmp_a+90)*math.pi/180),
                    ROBOT_HEIGHT/5)
    rstick.axis = axis
    rstick.size = (40,ROBOT_WIDTH-10+rstick_deployed, 20)
    rstick.color = (0.9, 0.2, 0.2)

	# Arm
    arm.pos = (tmp_x + ROBOT_X_OFFSET + (arm_offset * 15) * math.cos((tmp_a)*math.pi/180) + ((ROBOT_LENGTH/3 -20) * math.sin((tmp_a)*math.pi/180)),
		    tmp_y + (arm_offset * 15) * math.sin((tmp_a)*math.pi/180) - ((ROBOT_LENGTH/3 - 20) * math.cos((tmp_a)*math.pi/180)),
		    ROBOT_HEIGHT/2)
    arm.axis = (math.cos(tmp_a*math.pi/180+40*math.pi/180),
            math.sin(tmp_a*math.pi/180+40*math.pi/180),
            0)

    arm.size = (220, 40, 20)

    harvester.pos = (tmp_x + ROBOT_X_OFFSET + (harvester_offset * 60) * math.cos((tmp_a)*math.pi/180),
		    tmp_y + (harvester_offset * 60) * math.sin((tmp_a)*math.pi/180),
		    ROBOT_HEIGHT/4)

    harvester.axis = axis 
    """
    # save position
    save_pos.append((robot.pos.x, robot.pos.y, tmp_a))

    pos = robot.pos.x, robot.pos.y, 0.3
    if pos != last_pos:
        robot_trail_list.append(pos)
        last_pos = pos
    robot_trail_l = len(robot_trail_list)
    if robot_trail_l > max_trail:
        robot_trail_list = robot_trail_list[robot_trail_l - max_trail:]
    robot_trail.pos = robot_trail_list


def set_steam_shovel():
    global steam_shovel_rotateZ, steam_shovel_offset

    if steam_shovel_offset == 0:
        steam_shovel_offset = 6
    else:
	steam_shovel_offset = 0	

def set_robot2():
    global robot2, last_pos2, robot2_trail, robot2_trail_list
    global save_pos2, robot2_x, robot2_y, robot2_a
    global steam_shovel_rotateZ, steam_shovel_offset
    if color == YELLOW:
        tmp_x = robot2_x - AREA_X/2
        tmp_y = robot2_y - AREA_Y/2
        tmp_a = robot2_a
    else:
        tmp_x = -robot2_x + AREA_X/2
        tmp_y = -robot2_y + AREA_Y/2
        tmp_a = robot2_a

    robot2.pos = (tmp_x, tmp_y, ROBOT_HEIGHT/2)
    axis = (math.cos(tmp_a*math.pi/180),
            math.sin(tmp_a*math.pi/180),
            0)

    robot2.axis = axis
    robot2.size = (ROBOT2_LENGTH, ROBOT2_WIDTH, ROBOT_HEIGHT)
    
    """
    steam_shovel.pos = (tmp_x + (steam_shovel_offset * 15) * math.cos((tmp_a)*math.pi/180),
		    tmp_y + (steam_shovel_offset * 15) * math.sin((tmp_a)*math.pi/180),
    ROBOT_HEIGHT/6)
    steam_shovel.axis = (math.cos(tmp_a*math.pi/180),
            math.sin(tmp_a*math.pi/180),
            0)
    steam_shovel.size=(70, ROBOT2_WIDTH, 15)
    """

    # save position
    save_pos2.append((robot2.pos.x, robot2.pos.y, tmp_a))

    pos2 = robot2.pos.x, robot2.pos.y, 0.3
    if pos2 != last_pos2:
        robot2_trail_list.append(pos2)
        last_pos2 = pos2
    robot2_trail_l = len(robot2_trail_list)
    if robot2_trail_l > max_trail:
        robot2_trail_list = robot2_trail_list[robot2_trail_l - max_trail:]
    robot2_trail.pos = robot2_trail_list

def graph():
    pass

def save():
    f = open("/tmp/robot_save", "w")
    for p in save_pos:
        f.write("%f %f %f\n"%(p[0], p[1], p[2]))
    f.close()

def silent_mkfifo(f):
    try:
        os.mkfifo(f)
    except:
        pass

toggle_obj_disp()

while True:

    silent_mkfifo("/tmp/.robot_sim2dis")
    silent_mkfifo("/tmp/.robot_dis2sim")
    silent_mkfifo("/tmp/.robot2_sim2dis")
    silent_mkfifo("/tmp/.robot2_dis2sim")

    while True:
        fr = open("/tmp/.robot_sim2dis", "r")
        fw = open("/tmp/.robot_dis2sim", "w", 0)
        fr2 = open("/tmp/.robot2_sim2dis", "r")
        fw2 = open("/tmp/.robot2_dis2sim", "w", 0)

        while True:
            # MAIN ROBOT MSGS
            m = None
            l = fr. readline()
            l2 = fr2. readline()

            # parse position
            if not m:
                m = re.match("pos=%s,%s,%s"%(INT,INT,INT), l)
                if m:
                    robot_x = int(m.groups()[0])
                    robot_y = int(m.groups()[1])
                    robot_a = int(m.groups()[2])                  
                    set_robot()
                    # XXX HACK, send pos robot mate
                    #fw2.write("r2nd %d %d %d"%(int(robot_x), int(robot_y), int(robot_a)))
                    
                m = re.match("pos=%s,%s,%s"%(INT,INT,INT), l2)
                if m:
                    robot2_x = int(m.groups()[0])
                    robot2_y = int(m.groups()[1])
                    robot2_a = int(m.groups()[2])
                    set_robot2()
                    # XXX HACK, send pos robot mate
                    #fw.write("r2nd %d %d %d"%(int(robot2_x), int(robot2_y), int(robot2_a)))
            """
            # TODO parse slavedspic
            if not m:
                m = re.match("ballboard=%s"%(INT), l)
                if m:
                    print "ballboard: %d"%(int(m.groups()[0]))

            # parse cobboard
            if not m:
                m = re.match("cobboard=%s,%s"%(INT,INT), l)
                if m:
                    print "cobboard: %x,%x"%(int(m.groups()[0]),int(m.groups()[1]))
                    side = int(m.groups()[0])
                    flags = int(m.groups()[1])
                    if (side == 0 and color == YELLOW) or (side == 1 and color == GREEN):
                        robot_lstick_deployed = ((flags & 1) * 2)
                        robot_lstick_autoharvest = ((flags & 2) != 0)
                    else:
                        robot_rstick_deployed = ((flags & 1) * 2)
                        robot_rstick_autoharvest = ((flags & 2) != 0)
            """
            # DISPLAY EVENTS
            if scene.mouse.events != 0:
                if set_opp_nb == 1:
                    oppx, oppy, oppz = scene.mouse.getevent().project(normal=(0,0,1))
                    set_opp(oppx, oppy)
                    try:
                        if color == YELLOW:
                          fw.write("opp_1 %d %d"%(int(oppx + 1500), int(oppy + 1050)))
                          fw2.write("opp_1 %d %d"%(int(oppx + 1500), int(oppy + 1050)))
                        else:
                          fw.write("opp_1 %d %d"%(int(1500 - oppx), int(1050 - oppy)))
                          fw2.write("opp_1 %d %d"%(int(1500 - oppx), int(1050 - oppy)))
                    except:
                        print "not connected"
                else:
                    opp2x, opp2y, opp2z = scene.mouse.getevent().project(normal=(0,0,1))
                    set_opp2(opp2x, opp2y)
                    try:
                        if color == YELLOW:
                            fw.write("opp_2 %d %d"%(int(opp2x + 1500), int(opp2y + 1050)))
                            fw2.write("opp_2 %d %d"%(int(opp2x + 1500), int(opp2y + 1050)))
                        else:
                            fw.write("opp_2 %d %d"%(int(1500 - opp2x), int(1050 - opp2y)))
                            fw2.write("opp_2 %d %d"%(int(1500 - opp2x), int(1050 - opp2y)))
                    except:
                        print "not connected"

            if scene.kb.keys == 0:
                continue

            k = scene.kb.getkey()
            x,y,z = scene.center
            if k == "left":
                scene.center = x-10,y,z
            elif k == "right":
                scene.center = x+10,y,z
            elif k == "up":
                scene.center = x,y+10,z
            elif k == "down":
                scene.center = x,y-10,z
            #elif k == "l":
            #    fw.write("l")
            #elif k == "r":
            #    fw.write("r")
            #elif k == "b":
            #    fw.write("b")
            elif k == "c":
                robot_trail_list = []
                robot2_trail_list = []
            elif k == "x":
                save_pos = []
            elif k == "g":
                graph()
            elif k == "s":
                save()
            elif k == "h":
                toggle_obj_disp()
            elif k == "i":
                toggle_color()
	    elif k == "a":
		set_arm()
	    elif k == "r":
		set_lstick()
	    elif k == "l":
		set_rstick()
	    elif k == "p":
		set_steam_shovel()
	    elif k == "f":
		set_harvester()
            elif k == "1":
                set_opp_nb = 1;
            elif k == "2":
                set_opp_nb = 2;

            else:
                print k

            # EOF
            if l == "":
                break

        fr.close()
        fw.close()

