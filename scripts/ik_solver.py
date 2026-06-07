import math
import rclpy

from fk import fk_solve
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA


rclpy.init()

node = Node("ik_solver")

#Publisher for target marker in RViz
marker_publisher = node.create_publisher(
    Marker,
    '/target_marker',
    10
)
# Publisher for joint position commands '/position_controller/commands'
gazebo_publisher = node.create_publisher(
    Float64MultiArray,
    '/position_controller/commands',
    10
)


# Inverse Kinematics solver for 3DOF robot
def ik_solve(x, y):

    L1 = 0.5
    L2 = 0.4
    L3 = 0.3

    # Orientation of End Effector relative to world frame
    phi = 0.0

    # wrist x,y position
    wx = x - L3 * math.cos(math.radians(phi))
    wy = y - L3 * math.sin(math.radians(phi))
    r1 = math.sqrt(wx * wx + wy * wy)

    #To check whether the click is Too far or Too close
    print("Distance to wrist =", r1) 

    # Target is outside robot workspace (there is also a small forbidden circle near the base.)
    if r1 > (L1 + L2) or r1< abs(L1-L2):
        print()
        print("Target Unreachable !!!")
        print("Distance to wrist =", r1)
        print("Maximum Reach =", L1 + L2)
        print()
        return

    phi2 = math.atan2(wy, wx)

    # Law of cosines
    phi1 = math.acos(
        (L1**2 + r1**2 - L2**2) /
        (2 * L1 * r1)
    )

    theta1 = math.degrees(phi2) - math.degrees(phi1)

    # Law of cosines
    phi3 = math.acos(
        ((L1**2 + L2**2) - r1**2) /
        (2 * L1 * L2)
    )

    theta2 = 180 - math.degrees(phi3)

    theta3 = phi - theta1 - theta2

    # Convert joint angles from degrees to radians for ROS
    t1 = math.radians(theta1)
    t2 = math.radians(theta2)
    t3 = math.radians(theta3)

    # Values shown in the terminal
    print()

    print("Theta1 = ", theta1)

    print("Theta2 = ", theta2)

    print("Theta3 = ", theta3)

    print("phi1 = ", math.degrees(phi1))
    print("phi2 = ", math.degrees(phi2))

    return t1, t2, t3


# Called whenever a point is clicked in RViz
def callback(msg):

    # Target position from RViz
    x = msg.point.x
    y = msg.point.y


    #........Create target marker............(it just a marker for rviz Don't be afraid of this much code)

    marker = Marker()

    marker.header.frame_id = "base"
    marker.header.stamp = node.get_clock().now().to_msg()

    marker.ns = "target"
    marker.id = 0

    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    #Set Position
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.0
    marker.pose.orientation.w = 1.0

    #Set Size
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05

    #Set Color
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    #........................................

    # ........................................
    # Compute joint angles using IK
    result = ik_solve(x, y)

    # Ignore unreachable targets
    if result is None:
        return

    t1, t2, t3 = result

    # ........................................

    # Verifying the solved Theta's of IK using FK
    fx, fy, fphi = fk_solve(
        t1,
        t2,
        t3
    )

    # Displays clicked point on RViz and FK result for verification
    print()

    print("Clicked Point")
    print("X =", x)
    print("Y =", y)

    print()

    print("FK Result")
    print("X =", fx)
    print("Y =", fy)
    print("phi =", fphi)

    print()

    # Create controller command message
    gazebo_msg = Float64MultiArray()

    # Joint angles sent to controller
    gazebo_msg.data = [
        t1,
        t2,
        t3
    ]

    print("publishing...")
    print(gazebo_msg.data)

    marker_publisher.publish(marker)
    gazebo_publisher.publish(gazebo_msg)

    print("Robot Moving...")


# Subscribe to RViz clicked points
subscription = node.create_subscription(
    PointStamped,
    '/clicked_point',
    callback,
    10
)


# Keep node running and listening for clicks
rclpy.spin(node)

node.destroy_node()

rclpy.shutdown()