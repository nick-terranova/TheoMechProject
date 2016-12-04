'''
projectpart4.py
by Nick Terranova and Ted Morin

Demonstrates the difference between passive and semi-active suspension in a Unicycle.
'''

'''
def ground(x):
    if x < 1:
        return .15*x**.5
    else :
        return .15*x**2
    #return .15*x**2


ground_cyls = []
for i in range(150):
   t x = .01*i
    ground_cyls.append(cylinder(pos=(x,ground(x),-.5),axis=(0,0,1),radius=.01, color = color.green))
'''


from visual import *
#import sympy as syn
import numpy as np

# Note from Dr. Olenick: Compare with a paper.  (Use the active suspension)

class bike:
    # recommended wheel_rad-bridge_thick ratio = 5
    def __init__(self, pos=(0,-2,0), forward = (1,0,0),up=(0,1,0), wheel_thick = .3, 
            wheel_rad = .5 , bridge_thick=.1, beta=1.0, k=1.0, seat_mass=90.7):
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
        self.beta = float(beta)
        self.k = float(k)
        self.spr_len = 1.5*self.wheel_rad
        self.susp_rad = .5*self.bridge_thick

        self.forVel = 0.    # forward velocity
        self.seatVel = 0.   # velocity of seat, susp_upper w/ resp. to base

        # makes the wheel
        self.wheel = cylinder(pos=self.pos+self.wheel_rad*self.up-.5*self.wheel_thick*self.right,
            axis = self.wheel_thick * self.right, radius = self.wheel_rad, color = color.blue)

        # makes the frame
        self.bridge = box(pos = self.pos + (2*self.wheel_rad+1.5*self.bridge_thick) * self.up, 
            length = self.bridge_thick, width = 2*self.bridge_thick + self.wheel_thick, 
            height = self.bridge_thick)
        leg_height = (self.wheel_rad + 2.5*self.bridge_thick)
        self.left_leg = box(pos = self.pos + (self.wheel_rad+(leg_height-self.bridge_thick)/2) 
            * self.up - .5* (self.wheel_thick + self.bridge_thick) * self.right , 
            width = self.bridge_thick, length=self.bridge_thick, height = leg_height)
        self.right_leg = box(pos = self.pos + (self.wheel_rad+(leg_height-self.bridge_thick)/2) 
            * self.up + .5* (self.wheel_thick + self.bridge_thick) * self.right , 
            width = self.bridge_thick, length=self.bridge_thick, height = leg_height)

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
            make_trail=True) #, trail_radius = .2*seat_side, trail_color=color.red)
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
        d_pos = self.forVel*dt*self.forward
        self.pos += d_pos
        self.left_leg.pos += d_pos
        self.right_leg.pos += d_pos
        self.bridge.pos += d_pos
        self.wheel.pos += d_pos
        self.spring.pos += d_pos
        self.susp_lower.pos += d_pos
        self.susp_upper.pos += d_pos
        self.seat.pos += d_pos

    def bump(self,dh):
        d_pos = dh * self.up
        self.left_leg.pos += d_pos
        self.right_leg.pos += d_pos
        self.bridge.pos += d_pos
        self.wheel.pos += d_pos
        self.spring.pos += d_pos
        self.susp_lower.pos += d_pos
        self.spring.axis -= d_pos       # compress the spring!

dt = .01

bike_x = -9
bike_x_vel = 5.0
#spring_length = 2.0
k = 400.0 # spring constant
bump_height = 0.7

selected = False
while selected == False:
    dampbutton = input("Choose what type of damping you would like (0 : under,1 : critical,2 : over):")
    if dampbutton == 1:
        beta = np.sqrt(k/90.7) # damping parameter (equal to omega0)
        selected = True
    elif dampbutton == 0:
        beta = np.sqrt(k/90.7)-1.5 # damping parameter (equal to omega0)
        selected = True
    elif dampbutton == 2:
        beta = np.sqrt(k/90.7)+1.5 # damping parameter (equal to omega0)
        selected = True
    else:
        print "This is not a valid input.  Please try again."
    
    
# make the ground
ground1 = box(pos = (0,-.5,0),length = 50.0, width =6.0, height = 1.0)
ground2 = box(pos = (10,-.5+bump_height,0),length = 30.0, width =6.0, height = 1.0)
#groundtype = input("What type of bump would you like (square = 1, round = 2)?")
#if groundtype == 1:
#
    
"""
# make the bike
wheel = cylinder(pos=(bike_x,1,-0.5),axis=(0,0,1),radius=1, color = color.green)
wheelrod1 = box( pos = (bike_x, 2, .75)  , length = .5, width = .25, height = 2.0)
wheelrod2 = box( pos = (bike_x, 2, -.75) , length = .5, width = .25, height = 2.0)
crossbar = box ( pos = (bike_x, 2.75, 0) , length = .5, width = 1.5, height = .5)
spring = helix(pos=(-9,3,0),axis=(0,spring_length,0),radius=0.5, color = color.yellow)
seat = box( pos = (bike_x,  5.25, 0) , lenght = 2.0, width = .5, height = .5, make_trail=True)

# initialize the velocity of the seat in the bike's reference frame
seat.vel = vector(0,0,0)
seat.mass = 90.7 #in kilograms

# function to increment the bikes position (all parts()
def move_bike(d_pos = (dt*bike_x_vel,0,0)):
    vel = vector(d_pos)
    wheel.pos += d_pos
    wheelrod1.pos += d_pos
    wheelrod2.pos += d_pos
    crossbar.pos += d_pos
    spring.pos += d_pos
    seat.pos += d_pos

# function to increment the bikes position (all parts()
def move_bike_bump(d_pos = (dt*bike_x_vel,0,0)):
    vel = vector(d_pos)
    wheel.pos += d_pos
    wheelrod1.pos += d_pos
    wheelrod2.pos += d_pos
    crossbar.pos += d_pos
    spring.pos += d_pos

# takes the current spring length s as input, and computes
# acceleration, velocity, and new position of the seat
def oscillate(s):
    a = ((k/seat.mass)*(spring_length-s) - 2*beta*seat.vel.y)*vector(0,1,0)
    seat.vel += a*dt
    seat.pos += seat.vel*dt
    spring.axis += seat.vel*dt
"""
print beta 

el_biko = bike(pos = (bike_x, 0, 0), wheel_thick = .5, 
            wheel_rad = 1.2 , bridge_thick=.3, beta=beta, k=k, seat_mass=90.7)
el_biko.setForVel(bike_x_vel)

bumped = False
while el_biko.pos.x < 25:
    if not bumped and el_biko.pos.x > -5:
        el_biko.move(dt)
        el_biko.bump(bump_height)
        bumped = True
    el_biko.move(dt)
    el_biko.oscillate(dt)
    rate(50)




