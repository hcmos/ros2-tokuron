import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math
import qwiic_otos
import sys
import time

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('qwiic_odometry_node')

        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)   # 100Hz

        # rosparam
        self.declare_parameter('attached_pose', [0.0, 0.0, 0.0])
        attached_pose = self.get_parameter('attached_pose').get_parameter_value().double_array_value
        self.get_logger().info("attached pose : "+str(attached_pose))

        # qwiicの較正
        self.myOtos = qwiic_otos.QwiicOTOS()

        if self.myOtos.is_connected() == False:
            self.get_logger().info("The device isn't connected to the system. Please check your connection", \
                file=sys.stderr)
            return

        self.myOtos.begin()

        for i in range(5, 0, -1):
            self.get_logger().info("Calibrating in %d seconds..." % i)
            time.sleep(1)

        self.get_logger().info("Calibrating IMU...")

        self.myOtos.calibrateImu()
        self.myOtos.setLinearUnit(self.myOtos.kLinearUnitMeters)
        self.myOtos.setAngularUnit(self.myOtos.kAngularUnitRadians)

        offset = qwiic_otos.Pose2D(0.0, 0.030, 0.0)
        self.myOtos.setOffset(offset)

        self.myOtos.resetTracking()

        current = qwiic_otos.Pose2D(0.0, 0.0, 0.0)
        self.myOtos.setPosition(current)

        self.get_logger().info("Started Node")

    def timer_callback(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        current = self.myOtos.getPosition()

        odom.pose.pose.position.x = current.x
        odom.pose.pose.position.y = current.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation  = self.yaw_to_quaternion(current.h)

        # 出版
        self.publisher_.publish(odom)

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.w = math.cos(yaw / 2)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2)
        return q

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
