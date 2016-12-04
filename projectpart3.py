'''
projectpart3.py
'''


from visual import *
#import sympy as syn
import numpy as np

# Note from Dr. Olenick: Compare with a paper.  (Use the active suspension)

dt = .01

bike_x = -9
bike_x_vel = 5.0
spring_length = 2.0
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

print beta 

bumped = False
while wheel.x < 25:
    if not bumped and wheel.x > -5:
        dy = vector(0,bump_height,0)
        move_bike_bump(d_pos = dy)
        spring.axis -= dy
        bumped = True
    move_bike()
    oscillate(spring.axis.y)
    rate(50)




