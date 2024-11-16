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
        super().__init__('odometry_publisher')

        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 0.1秒間隔 (10Hz)


        # qwiicの較正
        print("\nQwiic OTOS Example 2 - Set Units\n")
        self.myOtos = qwiic_otos.QwiicOTOS()

        if self.myOtos.is_connected() == False:
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

        self.myOtos.resetTracking()

    def timer_callback(self):
        current_time = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'  # 親フレームID
        odom.child_frame_id = 'base_link'  # 子フレームID

        myPosition = self.myOtos.getPosition()

        odom.pose.pose.position.x = myPosition.x
        odom.pose.pose.position.y = myPosition.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation  = self.yaw_to_quaternion(myPosition.h)

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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
