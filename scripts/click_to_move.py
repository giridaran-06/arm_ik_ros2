                  
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64MultiArray
import math

class ClickListener(Node):

    def __init__(self):
        super().__init__('click_listener')

        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.callback,
            10
        )
        self.gazebo_publisher = self.create_publisher(
            Float64MultiArray,
            '/position_controller/commands',
             10
        )

    def callback(self,msg):

        x = msg.point.x
        y = msg.point.y

        print()
        print("Clicked:",x,y)

        L1 = 0.5
        L2 = 0.4
        L3 = 0.3

        phi = 0.0

        wx = x - L3*math.cos(math.radians(phi))
        wy = y - L3*math.sin(math.radians(phi))

        r1 = math.sqrt(wx*wx + wy*wy)

        if r1 > (L1 + L2):
            print("Unreachable")
            return

        phi2 = math.atan2(wy,wx)

        phi1 = math.acos(
            (L1**2 + r1**2 - L2**2)
            /(2*L1*r1)
        )

        theta1 = math.degrees(phi2) - math.degrees(phi1)

        phi3 = math.acos(
            ((L1**2 + L2**2) - r1**2)
            /(2*L1*L2)
        )

        theta2 = 180 - math.degrees(phi3)

        theta3 = phi - theta1 - theta2

        t1 = math.radians(theta1)
        t2 = math.radians(theta2)
        t3 = math.radians(theta3)

        cmd = Float64MultiArray()

        cmd.data = [
            t1,
            t2,
            t3
        ]

        self.gazebo_publisher.publish(cmd)

        print("Moved robot")


rclpy.init()

node = ClickListener()

rclpy.spin(node)


