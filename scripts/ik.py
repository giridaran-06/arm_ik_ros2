import math


L1 = 0.5
L2 = 0.4
L3 = 0.3
x = 0.6
y = 0.3
z = 0.0
phi = 30

wx = x - L3*math.cos(math.radians(phi))
wy = y - L3*math.sin(math.radians(phi))



r1 =math.sqrt(wx*wx + wy*wy)
if r1 > (L1+L2):
    print("Target Unreachable !!!")

phi2=math.atan2(wy,wx)

phi1=math.acos((L1**2 + r1**2 -L2**2)/(2*L1*r1))

theta1=math.degrees(phi2) - math.degrees(phi1)

phi3 = math.acos(((L1**2 +L2**2)-r1**2)/(2*L1*L2))

theta2 = 180 - math.degrees(phi3)

theta3 = phi - theta1 -theta2

print()
print("Theta1 = ",theta1)

print("Theta2 = ",theta2)

print("Theta3 = ",theta3)


print("phi1 = ",math.degrees(phi1))
print("phi2 = ",math.degrees(phi2))




