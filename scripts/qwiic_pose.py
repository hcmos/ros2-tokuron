import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
import math
import qwiic_otos
import sys
import time

class PoseStampedPublisher(Node):
    def __init__(self):
        super().__init__('qwiic_pose_node')

        self.publisher_ = self.create_publisher(PoseStamped, 'self_pose', 10)
        self.timer = self.create_timer(0.01, self.timer_callback)   # 100Hz

        # qwiicの較正
        print("\nQwiic OTOS Example 2 - Set Units\n")
        self.myOtos = qwiic_otos.QwiicOTOS()

        if not self.myOtos.is_connected():
            print("The device isn't connected to the system. Please check your connection", \
                file=sys.stderr)
            return

        self.myOtos.begin()

        print("Ensure the OTOS is flat and stationary during calibration!")
        for i in range(5, 0, -1):
            print("Calibrating in %d seconds..." % i)
            time.sleep(1)

        print("Calibrating IMU...")

        self.myOtos.calibrateImu()
        self.myOtos.setLinearUnit(self.myOtos.kLinearUnitMeters)
        self.myOtos.setAngularUnit(self.myOtos.kAngularUnitRadians)

        offset = qwiic_otos.Pose2D(0.0, 0.015, 0.0)
        self.myOtos.setOffset(offset)

        self.myOtos.resetTracking()

        current = qwiic_otos.Pose2D(0.0, 0.0, 0.0)
        self.myOtos.setPosition(current)

        print("Started Node")

    def timer_callback(self):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = 'odom'

        current = self.myOtos.getPosition()

        pose_stamped.pose.position.x = current.x
        pose_stamped.pose.position.y = current.y
        pose_stamped.pose.position.z = 0.0

        pose_stamped.pose.orientation = self.yaw_to_quaternion(current.h)

        # 出版
        self.publisher_.publish(pose_stamped)

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.w = math.cos(yaw / 2)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2)
        return q

def main(args=None):
    rclpy.init(args=args)
    node = PoseStampedPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
