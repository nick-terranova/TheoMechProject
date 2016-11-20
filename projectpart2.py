'''
projectpart2.py
critically damped oscilator unicycle 
'''

from visual import *
#import sympy as syn
import numpy as np

dt = .01

bike_x = -9
bike_x_vel = 5.0
spring_length = 2.0
k = 400.0 # spring constant
beta = np.sqrt(k/90.7) # damping parameter (equal to omega0)


# make the ground
ground = box(pos = (0,-.5,0),length = 50.0, width =6.0, height = 1.0)

# make the bike
wheel = cylinder(pos=(bike_x,1,-0.5),axis=(0,0,1),radius=1, color = color.green)
wheelrod1 = box( pos = (bike_x, 2, .75)  , length = .5, width = .25, height = 2.0)
wheelrod2 = box( pos = (bike_x, 2, -.75) , length = .5, width = .25, height = 2.0)
crossbar = box ( pos = (bike_x, 2.75, 0) , length = .5, width = 1.5, height = .5)
spring = helix(pos=(-9,3,0),axis=(0,spring_length,0),radius=0.5, color = color.yellow)
seat = box( pos = (bike_x,  5.25, 0) , lenght = 2.0, width = .5, height = .5, make_trail=True)

# initialize the velocity of the seat in the bike's reference frame
seat.vel = vector(0,-10,0)
seat.mass = 90.7 #in kilograms

# function to increment the bikes position (all parts()
def move_bike(vel = (dt*bike_x_vel,0,0)):
    vel = vector(vel)
    wheel.pos += vel
    wheelrod1.pos += vel
    wheelrod2.pos += vel
    crossbar.pos += vel
    spring.pos += vel
    seat.pos += vel

# takes the current spring length s as input, and computes
# acceleration, velocity, and new position of the seat
def oscillate(s):
    a = ((k/seat.mass)*(spring_length-s) - 2*beta*seat.vel.y)*vector(0,1,0)
    seat.vel += a*dt
    seat.pos += seat.vel*dt
    spring.axis += seat.vel*dt

print beta, 

while wheel.x < 25:
    move_bike()
    oscillate(spring.axis.y)
    rate(50)


