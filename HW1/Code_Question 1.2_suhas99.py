# Python Program to derive Kinematic Equations for 3 DOF Manipulator 

from sympy import *

x = symbols("x")              # x-coordinate of end effector
y = symbols("y")              # y-coordinate of end effector
phi = symbols("phi")          # Orientation of end effector
t = symbols("t")              # Time variable
l1 = symbols("l1")            # length of link 1
l2 = symbols("l2")            # length of link 2
l3 = symbols("l3")            # length of link 3
t1 = symbols("t1")            # Angle of link 1 wrt x-axis (base)
t2 = symbols("t2")            # Angle of link 2 wrt link 1
t3 = symbols("t3")            # Angle of link 3 wrt link 2
l1_dot = symbols("l1_dot")    # linear velocity of prismatic joint at link 1
t1_dot = symbols("t1_dot")    # angular velocity of revolute joint at link 1
t3_dot = symbols("t3_dot")    # angular velocity of revolute joint at link 3

# Initialization of variables as a function of time

l1 = Function('l1')(t)        # initialization of l1 as a function of time
t1 = Function('t1')(t)        # initialization of theta 1 as a function of time
t3 = Function('t3')(t)        # initialization of theta 3 as a function of time

# Velocities of different joints

l1_dot = diff(l1,t)           # linear velocity of prismatic joint between link 1 and 2
t1_dot = diff(t1,t)           # angular velocity of revolute joint between link 1 and base
t3_dot = diff(t3,t)           # angular velocity of revolute joint between link 2 and 3

# Position and Orientation Equations

x = (l1 * cos(t1)) + (l2 * cos(t1 + t2)) + (l3 * cos(t1 + t2 + t3))
y = (l1 * sin(t1)) + (l2 * sin(t1 + t2)) - (l3 * sin(t1 + t2 + t3))
phi = t1 + t2 + t3  # orientation of the end effector about x-axis

# Velocity Equations

x_dot = expand(expand_trig(expand(diff(x,t))))      # linear velocity of end effector along x-axis
y_dot = expand(expand_trig(expand(diff(y,t))))      # linear velocity of end effector along y-axis
phi_dot = diff(phi,t)                               # angular velocity of end effector about z-axis

# To create a Jacobian Matrix of the form [[a,b,c],[d,e,f],[g,h,i]]

a=x_dot.coeff(l1_dot)
b=x_dot.coeff(t1_dot)
c=x_dot.coeff(t3_dot)
d=y_dot.coeff(l1_dot)
e=y_dot.coeff(t1_dot)
f=y_dot.coeff(t3_dot)
g=phi_dot.coeff(l1_dot)
h=phi_dot.coeff(t1_dot)
i=phi_dot.coeff(t3_dot)

J1=transpose(Matrix([a,b,c]))             # Row 1 of Jacobian Matrix
J2=transpose(Matrix([d,e,f]))             # Row 2 of Jacobian Matrix
J3=transpose(Matrix([g,h,i]))             # Row 3 of Jacobian Matrix

Jacobian = Matrix.vstack(J1,J2,J3)        # Matrix of the shape 3X3

Jacobian_inverse = Jacobian.inv()         # Inverse of Jacobian Matrix for Inverse Kinematics

# Printing results

print("\n The Velocity Equations are: \n\n  x_dot: ", x_dot, "\n\n y_dot: ", y_dot, "\n\n phi_dot: ",phi_dot)

print("\n The Jacobian Matrix is: \n\n",Jacobian)

print("\n The Inverse Jacobian Matrix is: \n\n",Jacobian_inverse)