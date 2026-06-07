import math


#Forward Kinematics of 3DOF robot
def fk_solve(theta1,theta2,theta3):

    L1=0.5
    L2=0.4
    L3=0.3

    #theta1 = math.radians(-28.336631597465978)
    #theta2 = math.radians(132.79856000014672)
    #theta3 = math.radians(-74.46192840268074)

    x =L1*math.cos(theta1) + L2*math.cos(theta1+theta2) + L3*math.cos(theta1+theta2+theta3)
    #x =L1*math.cos(theta1) + L2*math.cos(theta1+theta2)

    y =L1*math.sin(theta1) + L2*math.sin(theta1+theta2) + L3*math.sin(theta1+theta2+theta3)
    #y =L1*math.sin(theta1) + L2*math.sin(theta1+theta2)

    phi =math.degrees(theta1+theta2+theta3)

    
    #print("X = ",x)

    #print("Y=  ",y)

    #print("phi = ",phi)
    


    # Below are used for checking the values are crt while debugging
    # print("x1 = ",L1*math.cos(theta1))
    # print("y1 = ",L1*math.sin(theta1))

    #  print("X2 = ",L1*math.cos(theta1) + L2*math.cos(theta1+theta2))
    #  print("Y2 = ",L1*math.sin(theta1) + L2*math.sin(theta1+theta2))
    #  print("X3 = ",L1*math.cos(theta1) + L2*math.cos(theta1+theta2) + L3*math.cos(theta1+theta2+theta3))
    #  print("Y3 = ",L1*math.sin(theta1) + L2*math.sin(theta1+theta2) + L3*math.sin(theta1+theta2+theta3))

    # Return end effector position and orientation
    return x,y,phi