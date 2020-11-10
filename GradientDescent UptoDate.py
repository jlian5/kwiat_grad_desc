# Automatic Alignment system code
# Ravi Acharya 
# 05/10/2020 

import numpy as np
import math
import time
import pyvisa as visa
from ThorlabsPM100 import ThorlabsPM100
import thorlabs_apt as apt


#---------- Initialising Powermeter reading & USB connections -----------#
rm = visa.ResourceManager()
inst = rm.open_resource('USB0::0x1313::0x8078::P0025003::INSTR', timeout=0)
power_meter = ThorlabsPM100(inst=inst)

#----------- Initialising mirror motors----------------------------------#
upperTopSN = 27004949
upperBtmSN = 27004956
lowerTopSN = 27004948
lowerBtmSN =  2700495

upperTop = apt.Motor(upperTopSN)
upperBtm = apt.Motor(upperBtmSN)
lowerTop = apt.Motor(lowerTopSN)
lowerBtm = apt.Motor(lowerBtmSN)

home_velocity = 1 #homing velocity
home_position = 3 #homing offset

upperTop.set_move_home_parameters(apt.HOME_REV, apt.HOMELIMSW_REV, home_velocity, home_position)
upperBtm.set_move_home_parameters(apt.HOME_REV, apt.HOMELIMSW_REV, home_velocity, home_position)
lowerTop.set_move_home_parameters(apt.HOME_REV, apt.HOMELIMSW_REV, home_velocity, home_position)
lowerBtm.set_move_home_parameters(apt.HOME_REV, apt.HOMELIMSW_REV, home_velocity, home_position)
#---------- Performance checking model functions -----------#

def findplane(x,y,z,x0,y0,z0):
    # Denote x,y,z as the vector components for new n vector
    # Denote a,b,c,d as new vector coefficients
    a = x
    b = y
    c = z
    d = -((x*x0)+(y*y0)+(z*z0))
    return a,b,c,d

# Function to find the point of interaction given beam parameters
# Function returns point of intersection (x,y,z)
# Input is plane parameters and beam angles
def poi(a,b,c,d,theta,phi):
    py = math.sin(theta)*math.sin(phi)
    px = math.sin(theta)*math.cos(phi)
    pz = math.cos(theta)
    q = d/((a*px)+(b*py)+(c*pz))
    t = np.absolute(q)
    Ix = t*px
    Iy = t*py
    Iz = t*pz
    return Ix, Iy, Iz

# Function to rotate to new frame of reference
# Input: x,y,z is the point in non-rotated frame
# Beta and Theta are the rotations about the x and z-axes respectively
def introt(x,y,z,beta,theta):
    # Intial rotation of the n vectors
    xn = x*math.cos(beta)-(y*math.cos(theta)*math.sin(beta))+(z*math.sin(beta)*math.sin(theta))
    yn = x*math.sin(beta)+(y*math.cos(theta)*math.cos(beta))-(z*math.sin(theta)*math.cos(beta))
    zn = y*math.sin(theta)+z*math.cos(theta)
    return xn,yn,zn

# Function to rotate from new frame back to original frame
# Used to rotate off axis n-hat to be parallel to +ve y-axis
# Inverse of matrix in the function above
def inverse(x,y,z,beta,theta):
    # Inverse back to global
    xn = (x*math.cos(beta))+(y*math.sin(beta))
    yn = -(x*math.sin(beta)*math.cos(theta))+(y*math.cos(theta)*math.cos(beta))+(z*math.sin(theta))
    zn = (x*math.sin(beta)*math.sin(theta))-(y*math.cos(beta)*math.sin(theta))+(z*math.cos(theta))
    return xn, yn, zn 

# Function to rotate second mirror so that n-hat vector along +ve y-axis
def invflip(x,y,z,beta,theta):
    xn = (x*math.cos(beta))+(y*math.sin(beta))
    yn = (x*math.sin(beta)*math.cos(theta))-(y*math.cos(theta)*math.cos(beta))-(z*math.sin(theta))
    zn = (x*math.sin(theta)*math.sin(beta))-(y*math.cos(beta)*math.sin(theta))+(z*math.cos(theta))
    return xn,yn,zn

#Function to calculate spherical coordinates angles of point
def angle(x,y,z):
    r = np.sqrt((x*x)+(y*y)+(z*z))
    phi = -math.atan(y/x)
    theta = math.acos(z/r)
    return theta, phi

#--------------Performance checking model-------------------------------#


# Define intial mirror positions/angles and initial beam angle
# Assume beam is intially near parallel to table top
angles = [math.pi/2,0]
# Intial mirror slant and tilts
m1a = [math.pi/4,0]
m2a = [-math.pi/4,0]
# Start model off with n-hat vectors along the y-axis
n1 = [0,1,0]
n2 = [0,-1,0]
# Initial mirror centre positions relative to intial beam output  
m1 = [1,0,0]
m2 = [1,1,0]
# Final fiber position
m3 = [0,1,0]
a = m2[0]-m3[0]
b = m2[1]-m3[1]
# Mirror diameter
m1r = 1
m2r = 1

    
# Function to propagate intial beam angles to final beam angles in relative 
# frame of reference
# Input are the intial mirror slant and tilt angles
# Function returns overlap of beam given current angles
def main(m1a,m2a):
    global n1,n2,m1,m2,m3,m1r,m2r
    # Calculate intial n-hat vectors given intial mirror angles
    n1 = introt(n1[0],n1[1],n1[2],m1a[0],m1a[1])
    n2 = introt(n2[0],n2[1],n2[2],m2a[0],m2a[1])
    # Find plane parameters in global frame
    p1 = []
    p1 = findplane(n1[0],n1[1],n1[2],m1[0],m1[1],m1[2])
    # Calculate POI with mirror 1 in global frame
    xpg,ypg,zpg = poi(p1[0],p1[1],p1[2],p1[3],angles[0],angles[1])
    # Apply transformation to make n1 parallel to y-axis
    # i.e. move into frame of reference of POI 1
    xp1,yp1,zp1 = inverse(xpg,ypg,zpg,m1a[0],m1a[1])
    # Calculate incoming beam angles in POI 1 frame of reference
    theta1, phi1 = angle(xp1,yp1,zp1)
    # Calculate n-hat of mirror 2 and mirror 2 position in POI 1 frame of ref
    n2 = inverse(n2[0],n2[1],n2[2],m1a[0],m1a[1])
    m2 = inverse((m2[0]-xpg),(m2[1]-ypg),(m2[2]-zpg),m1a[0],m1a[1])
    # Calculate plane 2 and POI 2 in POI 1 f.o.r
    p2 =[]
    p2 = findplane(n2[0],n2[1],n2[2],m2[0],m2[1],m2[2])
    xp2,yp2,zp2 = poi(p2[0],p2[1],p2[2],p2[3],theta1,phi1)
    # Calculate POI 2 in global frame
    xp2ge,yp2ge,zp2ge = introt(xp2,yp2,zp2,m1a[0],m1a[1])
    xp2g = xpg + xp2ge
    yp2g = ypg + yp2ge
    zp2g = zpg + zp2ge
    # Calculate POI 1 in POI 2 f.o.r
    poi1r2 = []
    poi1r2 = invflip((xpg-xp2g),(ypg-yp2g),(zpg-zp2g),m2a[0],m2a[1])
    # Calculate final fiber position in POI 2 f.o.r
    m3r2 = invflip((m3[0]-xp2g),(m3[1]-yp2g),(m3[2]-zp2g),m2a[0],m2a[1])
    # Calculate  beam angles coming off mirror 2 in POI 2 f.o.r
    theta2, phi2 = angle(poi1r2[0],poi1r2[1],poi1r2[2])
    # Calculate angles of final fiber positon in POI 2 f.o.r
    theta3, phi3 = angle(m3r2[0],m3r2[1],m3r2[2])
    # Account for reflection
    phi3 = -phi3
    # Calculate overlap
    td = (theta3 - theta2)*(180/math.pi)
    pd = (phi3 - phi2)*(180/math.pi)
    overlap = math.exp((-(td*td)-(pd*pd))/4)
    return overlap


#----------------Gradient Descent Algorithm-------------------#


# Functions below are the partial derivatives of the overlap function w.r.t 
# tip/tilts of mirrors
# Input are the tips/tilts 
# Aim1 and Aim2 are the angles the algorithm aims for the slants of both mirrors
# The algorithm aims for tips of zero

def partialtheta1(t1,p1,t2,p2,aim1,aim2):
    b = 0.25*(-(t1-aim1)**2-(t2+aim2)**2-p1**2-p2**2)
    c = (t1-aim1)/2
    f = c*math.exp(b)
    return f

def partialphi1(t1,p1,t2,p2,aim1,aim2):
    b = 0.25*(-(t1-aim1)**2-(t2+aim2)**2-p1**2-p2**2)
    c = p1/2
    f = c*math.exp(b)
    return f

def partialtheta2(t1,p1,t2,p2,aim1,aim2):
    b = 0.25*(-(t1-aim1)**2-(t2+aim2)**2-p1**2-p2**2)
    c = (t2+aim2)/2
    f = c*math.exp(b)
    return f

def partialphi2(t1,p1,t2,p2,aim1,aim2):
    b = 0.25*(-(t1-aim1)**2-(t2+aim2)**2-p1**2-p2**2)
    c = p2/2
    f = c*math.exp(b)
    return f
       

# Intialise parameters
theta1, phi1 = m1a 
theta2, phi2 = m2a
# Alpha is a movement parameter assocaited with derivative
alpha = 2.0
iterations = 0
check = 0
# Define precision to stop algorithm
precision = 1/1000000
printData = True
# Maximum iterations at which algorithm will stop if stopping criterion not met
maxIterations = 100
# Initialise motor resolution (5.3 microrad in our case)
res = 5.3E-06

# Initialize arrays to collect values after each iteration

t1, t2, p1, p2 = [],[],[],[]
t1.append(theta1)
t2.append(theta2)
p1.append(phi1)
p2.append(phi2)
# Save the number steps the code takes
t1s,t2s,p1s,p2s = [],[],[],[]

# Calculate aim angles depending on intial mirror positions 
# Aim2 varies with position of final fiber position

aim1 = math.pi/4

if a == 0 or b == 0:
    aim2 = math.pi/4
else:
    aim2 = math.atan(a/b)/2
        

# Algorithm main loop
while True:
   # Use partial derivatives to change to new temp value
    temptheta1 = theta1 - alpha*partialtheta1(theta1,phi1,theta2,phi2,aim1,aim2)
    temptheta2 = theta2 - alpha*partialtheta2(theta1,theta1,theta2,phi2,aim1,aim2)
    tempphi1 = phi1 - alpha*partialphi1(theta1,phi1,theta2,phi2,aim1,aim2)
    tempphi2 = phi2 - alpha*partialphi2(theta1,phi1,theta2,phi2,aim1,aim2)
    # Adjust new values depending on resolution of motor
    i1 = (theta1 - temptheta1)/res
    i1 = round(i1,0)
    temptheta1 = theta1 - (i1*res)
    i2 = (theta2 - temptheta2)/res
    i2 = round(i2,0)
    temptheta2 = theta2 - (i2*res)
    i3 = (phi1 - tempphi1)/res
    i3 = round(i3,0)
    tempphi1 = phi1 - (i3*res)
    i4 = (phi2 - tempphi2)/res
    i4 = round(i4,0)
    tempphi2 = phi2 - (i4*res)
    # Append arrays
    t1.append(temptheta1)
    t2.append(temptheta2)
    p1.append(tempphi1)
    p2.append(tempphi2)
    t1s.append(i1)
    t2s.append(i2)
    p1s.append(i3)
    p2s.append(i4)
    #If the number of iterations goes up too much, maybe theta (and/or theta1)
    #is diverging
    iterations += 1
    if iterations > maxIterations:
        print("Too many iterations. Adjust alpha and make sure that the function is convex!")
        printData = False
        break

    #If the value of the parameters to be optimised changes less of a certain amount, our goal is met.
    if abs(temptheta1-theta1) < precision and abs(temptheta2-theta2) < precision and abs(tempphi1-phi1) < precision and abs(tempphi2-phi2) < precision:
        #printData = True
        break

    #Simultaneous update to new values
    theta1 = temptheta1
    theta2 = temptheta2
    phi1 = tempphi1
    phi2 = tempphi2
    
# Algorithm output    
if printData:
    # If algorithm has worked output final values
    m1f = [theta1, phi1]
    m2f = [theta2, phi2]
    f = (180/math.pi)
    overlap = main(m1f,m2f)
    t1d, p1d, t2d, p2d = (theta1*f),(phi1*f),(theta2*f),(phi2*f)
    print("Number of iterations:",iterations,sep=" ")
    print("theta1 =",t1d,sep=" ")
    print("theta2  =",t2d,sep=" ")
    print("phi1  =",p1d,sep=" ")
    print("phi2  =",p2d,sep=" ")
    print("Overlap = ", overlap, sep = " ")
   
    
    
    