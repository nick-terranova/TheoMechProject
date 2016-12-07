'''
projectpart4.py
by Nick Terranova and Ted Morin

Demonstrates the difference between passive and semi-active suspension in a Unicycle.
'''

from visual import *
#import sympy as syn
import numpy as np
import sys
import matplotlib.pyplot as plt

# Note from Dr. Olenick: Compare with a paper.  (Use the active suspension)

class Ground:
    def __init__(self, beg, end, step, setGround):
        # all of data about the ground...
        self.beg = float(beg)
        self.end = float(end)
        self.step = float(step)
        self.length = np.abs(self.end - self.beg)
        self.heights = []
        self.points  = []
        self.setGround = setGround

        # make the ground points and heights
        x = self.beg
        for i in range(int(self.length/self.step) + 1):
            x += self.step
            y = setGround(x)
            self.heights.append(y)
            new_point = (x,y,0)
            self.points.append(new_point)
        self.body = curve(pos=self.points)

    def getHeight(self, x):
        if x < self.beg :
            return self.heights[0]
        elif x > self.end:
            return self.heights[-1]
        else :
            x = x / self.step # convert to units of "ground-steps"
            i = int(x)
            return self.heights[i] + (x - i) * (self.heights[i+1] - self.heights[i])

class Bike:
    # recommended wheel_rad-bridge_thick ratio = 5
    def __init__(self, pos=(0,-2,0), forward = (1,0,0),up=(0,1,0), wheel_thick = .3, 
            wheel_rad = .5 , bridge_thick=.1, b=1.0, k=1.0, seat_mass=90.7):
        self.pos = vector(pos)
        # up unit vector is normalized
        self.up = vector(up).norm()                 
        # forward unit vector is normalized and orthogonal
        forward = vector(forward)
        self.forward = (forward - forward.proj(self.up)).norm()  
        # right unit vector is normalized implicitly
        self.right = self.forward.cross(self.up)

        # scalar constants
        self.wheel_thick = float(wheel_thick)
        self.wheel_rad = float(wheel_rad)
        self.bridge_thick = float(bridge_thick)
        self.b = float(b)
        self.beta = self.b/(2*seat_mass)
        self.k = float(k)
        self.spr_len = 1.5*self.wheel_rad
        self.susp_rad = .5*self.bridge_thick

        self.forVel = 0.    # forward velocity
        self.seatVel = 0.   # velocity of seat, susp_upper w/ resp. to base

        # makes the wheel
        self.wheel = cylinder(pos=self.pos+self.wheel_rad*self.up-.5*self.wheel_thick*self.right,
            axis = self.wheel_thick * self.right, radius = self.wheel_rad, color = (.7,.7,.7))

        # makes the frame
        self.bridge = box(pos = self.pos + (2*self.wheel_rad+1.5*self.bridge_thick) * self.up, 
            length = self.bridge_thick, width = 2*self.bridge_thick + self.wheel_thick, 
            height = self.bridge_thick, color=color.orange)
        leg_height = (self.wheel_rad + 2.5*self.bridge_thick)
        self.left_leg = box(pos = self.pos + (self.wheel_rad+(leg_height-self.bridge_thick)/2) 
            * self.up - .5* (self.wheel_thick + self.bridge_thick) * self.right , 
            width = self.bridge_thick, length=self.bridge_thick, height = leg_height,
            color=color.orange)
        self.right_leg = box(pos = self.pos + (self.wheel_rad+(leg_height-self.bridge_thick)/2) 
            * self.up + .5* (self.wheel_thick + self.bridge_thick) * self.right , 
            width = self.bridge_thick, length=self.bridge_thick, height = leg_height,
            color=color.orange)

        # makes the spring, lower suspension
        self.spring = helix(pos=self.pos+2*(self.wheel_rad + self.bridge_thick) * self.up,
            axis=self.spr_len*self.up, radius=1.2*self.susp_rad, thickness = .2*self.susp_rad, 
            color = color.yellow)
        self.susp_lower = cylinder(pos = self.spring.pos, axis = .5*self.spring.axis, 
            radius = .8*self.susp_rad, color = (.8,0,0) )

        # makes upper suspension, "seat"
        self.susp_upper = cylinder(pos = self.spring.pos + self.spring.axis, 
            axis = -.75*self.spring.axis, radius = self.susp_rad, color = (1,0,0) )
        seat_side = 2 * self.bridge_thick + self.wheel_thick
        self.seat = box(pos = self.susp_upper.pos + .5*seat_side*self.up,
            width = seat_side, height= seat_side, length = self.wheel_rad,
            make_trail=True, color = color.yellow)
        self.seat.mass = seat_mass

    # takes the differential equation for the acceleration of the passive system,
    # calculates the the distance the seat ought to be moved vertically, and moves the seat
    def oscillate(self, dt):
        a = (self.k/self.seat.mass)*(self.spr_len-self.spring.axis.mag) - 2*self.beta*self.seatVel
        self.seatVel += a*dt
        d_pos = self.seatVel * dt * self.up
        self.seat.pos += d_pos
        self.susp_upper.pos += d_pos
        self.spring.axis += d_pos
        return a

    # allows the main code to set the horizontal velocity of the unicycle
    def setForVel(self, vel):
        self.forVel = vel

    # moves the entire bike horizontally and moves all parts of the bike other
    # than the seat vertically to match the height of the ground at the point
    # that the bike just moved to
    def move(self, dt):
        dx = dt * self.forVel
        d_pos = dx*self.forward
        dy = ground.getHeight(self.pos.x+dx) - self.pos.y
        d_pos_base = d_pos + dy*self.up
        self.pos += d_pos_base
        self.left_leg.pos += d_pos_base
        self.right_leg.pos += d_pos_base
        self.bridge.pos += d_pos_base
        self.wheel.pos += d_pos_base
        self.spring.pos += d_pos_base
        self.susp_lower.pos += d_pos_base
        self.susp_upper.pos += d_pos
        self.seat.pos += d_pos
        self.spring.axis -= dy*self.up   # compress the spring!

# Definition of global constants
dt = .01                        # time step
seat_mass = 90.7                # seat mass
bike_x = -9                     # initial x-position of the bike
bike_x_vel = 4.0               # initial y-position of the bike
#spring_length = 2.0
k = 400.0                       # spring constant
groundlength = 1000.0             # length of the ground
groundstep = 0.05               # distance between points of the ground
rate_param = 10000
data_points = 600

# Asks for the type of damping the user wants to input
try :
    dampbutton = int(sys.argv[1])
    if dampbutton not in [0, 1, 2]:
        dampbutton = 1
except :
    dampbutton = 1

if dampbutton == 1:
    b = 2*np.sqrt(k*seat_mass) # damping parameter (equal to omega0)
    selected = True
elif dampbutton == 0:
    b = 2*np.sqrt(k*seat_mass)- 3*seat_mass # damping parameter (equal to omega0)
    selected = True
elif dampbutton == 2:
    b = 2*np.sqrt(k*seat_mass)+ 3*seat_mass # damping parameter (equal to omega0)
    selected = True
    

# make the ground
def g1(x):
    if x < 10 or x > 15:
        return x*0
    elif 10 <= x <= 15:
        return np.sin(2*np.pi*x/10)

def g2(x):
    return (-.5*np.sin(2*np.pi*x/300) + .05* np.sin(np.pi*5*x) + np.sin(2*np.pi*x/600) \
        + np.cos(2*np.pi*x/300) - np.cos(2*np.pi*x/900))*.2

def g3(x):
    return (-.5*np.sin(2*np.pi*x/100) + .02* np.sin(np.pi*x) + np.sin(2*np.pi*x/30) \
        + np.cos(2*np.pi*x/40) - np.cos(2*np.pi*x/30))*.2

f = lambda x: g3(x)

# Creates the objects of ground and bike classes
ground = Ground(0, groundlength, groundstep, f)
el_biko = Bike(pos = (0, 0, 0), wheel_thick = .5, 
            wheel_rad = 1.2 , bridge_thick=.3, b=b, k=k, seat_mass=seat_mass)
el_biko.setForVel(bike_x_vel)

# initializes lists for time and acceleration
acc = 0
currenttime = 0
tlist = [0.0]
acclist = [0.0]
acc_sum = 0
# Main program loop
#scene.forward = (-2,-1,-1)
scene.range = 10
while el_biko.pos.x < ground.length:
    # update bike
    el_biko.move(dt)
    acc = el_biko.oscillate(dt)

    # update data
    acclist.append(acc)
    acc_sum += np.abs(acc)
    currenttime += dt
    tlist.append(currenttime)

    # update the scene
    scene.center = el_biko.pos + 2*el_biko.wheel_rad*el_biko.up  # centers the bike in the window
    rate(rate_param)

print acc_sum/len(acclist)

tlistshort = []
acclistshort = []
for i in range(len(acclist)/10):
    tlistshort.append(tlist[10*i])
    acclistshort.append(acclist[10*i])


# Plots a graph of acceleration of the seat vs. time for the run
plt.plot(tlistshort,acclistshort,'ro')
plt.title("Acceleration of the Unicycle Seat vs. Time")
plt.xlabel("Time (s)")
plt.ylabel("Acceleration (m/s^2)")
plt.show()
