'''
projectpart4.py
by Nick Terranova and Ted Morin

Demonstrates the difference between passive and semi-active suspension in a Unicycle.
'''

from visual import *
#import sympy as syn
import numpy as np

# Note from Dr. Olenick: Compare with a paper.  (Use the active suspension)

class bike:
    # recommended wheel_rad-bridge_thick ratio = 5
    def __init__(self, pos=(0,-2,0), forward = (1,0,0),up=(0,1,0), wheel_thick = .3, 
            wheel_rad = .5 , bridge_thick=.1, b=1.0, k=1.0, seat_mass=90.7):
        self.pos = vector(pos)
        # up is normalized
        self.up = vector(up).norm()                 
        # forward is normalized and orthogonal
        forward = vector(forward)
        self.forward = (forward - forward.proj(self.up)).norm()  
        # right is normalized implicitly
        self.right = self.forward.cross(self.up)

        # non-vector constants
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

    def oscillate(self, dt):
        a = (self.k/self.seat.mass)*(self.spr_len-self.spring.axis.mag) - 2*self.beta*self.seatVel
        self.seatVel += a*dt
        d_pos = self.seatVel * dt * self.up
        self.seat.pos += d_pos
        self.susp_upper.pos += d_pos
        self.spring.axis += d_pos

    def setForVel(self, vel):
        self.forVel = vel

    def move(self, dt):
        dx = dt * self.forVel
        d_pos = dx*self.forward
        dy = getGround(self.pos.x+dx) - self.pos.y
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

dt = .01
seat_mass = 90.7
bike_x = -9
bike_x_vel = 7.50
#spring_length = 2.0
k = 400.0 # spring constant
bump_height = 0.8

selected = False
while selected == False:
    dampbutton = input("Choose what type of damping you would like (0 : under,1 : critical,2 : over):")
    if dampbutton == 1:
        b = 2*np.sqrt(k*seat_mass) # damping parameter (equal to omega0)
        selected = True
    elif dampbutton == 0:
        b = 2*np.sqrt(k*seat_mass)- 3*seat_mass # damping parameter (equal to omega0)
        selected = True
    elif dampbutton == 2:
        b = 2*np.sqrt(k*seat_mass)+ 3*seat_mass # damping parameter (equal to omega0)
        selected = True
    else:
        print "This is not a valid input.  Please try again."
    
    
# make the ground
#ground1 = box(pos = (0,-.5,0),length = 50.0, width =6.0, height = 1.0)
def getGround(x):
#    return np.sin(2*np.pi*x/10)
    if x < 10 or x > 15:
        return x*0
    elif 10 <= x <= 15:
        return np.sin(2*np.pi*x/10)
groundlength = 50.0
groundpoints = []
groundheight = []
groundstep = 0.05
current_x = 0
for i in range(int(groundlength/groundstep)):
    current_x += groundstep
    groundunit = (current_x,getGround(current_x),0)
    groundpoints.append(groundunit)
ground = curve(pos=groundpoints)

el_biko = bike(pos = (0, 0, 0), wheel_thick = .5, 
            wheel_rad = 1.2 , bridge_thick=.3, b=b, k=k, seat_mass=seat_mass)
el_biko.setForVel(bike_x_vel)

bumped = False
while el_biko.pos.x < groundlength:
    el_biko.move(dt)
    el_biko.oscillate(dt)
    rate(100)




