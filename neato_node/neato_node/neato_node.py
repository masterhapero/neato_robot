#!/usr/bin/python3

"""
ROS2 node for Neato robot vacuums.
"""

#__author__ = "jnugen@gmail.com (James Nugen)"

import rclpy
from rclpy.node import Node

from math import sin,cos,pi
import sys

from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from neato_msgs.msg import Button, Sensor
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from neato_driver.neato_driver import Botvac

class NeatoNode(Node):
    
    def __init__(self):
        """ Start up connection to the Neato Robot. """
        super().__init__('neato_node')

        self.CMD_RATE = 2

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('frame_id', 'base_laser_link')

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.get_logger().info('Using port: %s' % self.port)

        try:
            self.robot = Botvac(self.port)
        except:
            self.get_logger().error("Could not connect to Botvac")
            raise

        self.create_subscription(Twist, 'cmd_vel', self.cmdVelCb, 10)
        self.scanPub = self.create_publisher(LaserScan, 'base_scan', 10)
        self.odomPub = self.create_publisher(Odometry,'odom', 10)
        self.buttonPub = self.create_publisher(Button, 'button', 10)
        self.sensorPub = self.create_publisher(Sensor, 'sensor', 10)
        self.odomBroadcaster = TransformBroadcaster(self)

        self.cmd_vel = [0, 0]
        self.old_vel = self.cmd_vel

        self._spin_init()

    def _spin_init(self):
        self._encoders = [0, 0]

        self._x = 0.0                  # position in xy plane
        self._y = 0.0
        self._th = 0.0

        self._then = self.get_clock().now()

        # things that don't ever change
        scan_link = self.get_parameter('frame_id').get_parameter_value().string_value
        self._scan = LaserScan(header = Header(frame_id = scan_link))

        self._scan.angle_min = 0.0 
        self._scan.angle_max = 359.0 * pi / 180.0 
        self._scan.angle_increment = pi / 180.0 
        self._scan.range_min = 0.020
        self._scan.range_max = 5.0

        self._odom_ts = TransformStamped()
        self._odom_ts.header.frame_id = 'odom'
        self._odom_ts.child_frame_id = 'base_footprint'

        self._odom = Odometry()
        self._odom.header.frame_id = 'odom'
        self._odom.child_frame_id = 'base_footprint'

        self._button_enum = ("Soft_Button", "Up_Button", "Start_Button", "Back_Button", "Down_Button")
        self._sensor_enum = ("Left_Side_Bumper", "Right_Side_Bumper", "Left_Bumper", "Right_Bumper")

        self.robot.setBacklight(1)
        #self.robot.setLED("ledgreen") #doesn't exist on Botvac Connected

        # main loop timer
        self._cmd_rate = self.CMD_RATE
        timer_period = 1.0 / 20.0  # 20Hz
        self._timer = self.create_timer(timer_period, self._spin_cb)

    def _spin_cb(self):
        # notify if low batt
        #if self.robot.getCharger() < 10:
        #    print "battery low " + str(self.robot.getCharger()) + "%"

        # get motor encoder values
        left, right = self.robot.getMotors()

        self._cmd_rate -= 1
        if self._cmd_rate == 0:
            # send updated movement commands
            #if self.cmd_vel != self.old_vel or self.cmd_vel == [0,0]:
                # max(abs(self.cmd_vel[0]),abs(self.cmd_vel[1])))
            #self.robot.setMotors(self.cmd_vel[0], self.cmd_vel[1], (abs(self.cmd_vel[0])+abs(self.cmd_vel[1]))/2)
            self.robot.setMotors(self.cmd_vel[0], self.cmd_vel[1], max(abs(self.cmd_vel[0]), abs(self.cmd_vel[1])))
            self._cmd_rate = self.CMD_RATE

        self.old_vel = self.cmd_vel

        # prepare laser scan
        now = self.get_clock().now()
        self._scan.header.stamp = now.to_msg()

        self.robot.requestScan()
        self._scan.ranges, self._scan.intensities = self.robot.getScanRanges()

        # now update position information
        dt = (now - self._then).nanoseconds / 1.0e9
        self._then = now

        d_left = (left - self._encoders[0]) / 1000.0
        d_right = (right - self._encoders[1]) / 1000.0
        self._encoders = [left, right]
        #print(d_left, d_right, self._encoders)

        dx = (d_left + d_right) / 2
        dth = (d_right - d_left) / (self.robot.base_width / 1000.0)

        x = cos(dth) * dx
        y = -sin(dth) * dx
        self._x += cos(self._th) * x - sin(self._th) * y
        self._y += sin(self._th) * x + cos(self._th) * y
        self._th += dth
        #print(self.x,self.y,self.th)

        # prepare tf from base_link to odom
        quaternion = Quaternion()
        quaternion.z = sin(self._th / 2.0)
        quaternion.w = cos(self._th / 2.0)

        # prepare TransformStamped msg
        self._odom_ts.header.stamp = self.get_clock().now().to_msg() #now?
        self._odom_ts.transform.translation.x = self._x
        self._odom_ts.transform.translation.y = self._y
        self._odom_ts.transform.translation.z = 0.0
        self._odom_ts.transform.rotation = quaternion

        # prepare Odometry msg
        self._odom.header.stamp = self.get_clock().now().to_msg()
        self._odom.pose.pose.position.x = self._x
        self._odom.pose.pose.position.y = self._y
        self._odom.pose.pose.position.z = 0.0
        self._odom.pose.pose.orientation = quaternion
        self._odom.twist.twist.linear.x = dx / dt
        self._odom.twist.twist.angular.z = dth / dt

        # sensors
        lsb, rsb, lfb, rfb = self.robot.getDigitalSensors()

        # buttons
        btn_soft, btn_scr_up, btn_start, btn_back, btn_scr_down = self.robot.getButtons()

        # publish everything
        self.odomBroadcaster.sendTransform(self._odom_ts)
        self.scanPub.publish(self._scan)
        self.odomPub.publish(self._odom)

        button = Button()
        for idx, b in enumerate((btn_soft, btn_scr_up, btn_start, btn_back, btn_scr_down)):
            if b == 1:
                button.value = bool(b)
                button.name = self._button_enum[idx]
                self.buttonPub.publish(button)

        sensor = Sensor()
        for idx, b in enumerate((lsb, rsb, lfb, rfb)):
            if b == 1:
                sensor.value = bool(b)
                sensor.name = self._sensor_enum[idx]
                self.sensorPub.publish(sensor)

    def shutdown(self):
        self.destroy_timer(self._timer)
        self.robot.shutdown()

    def sign(self,a):
        if a >= 0:
            return 1
        else:
            return -1

    def cmdVelCb(self,req):
        x = req.linear.x * 1000
        th = float(req.angular.z * (self.robot.base_width / 2))
        k = max(abs(x-th),abs(x+th))
        # sending commands higher than max speed will fail

        if k > self.robot.max_speed:
            x = x*self.robot.max_speed/k; th = th*self.robot.max_speed/k

        self.cmd_vel = [int(x-th), int(x+th)]

def main(args=None):
    rclpy.init(args=args)

    exit_code = 0

    try:
        node = NeatoNode()
    except:
        exit_code = 1
    else:
        try:
            rclpy.spin(node)
        except:
            node.shutdown()
            node.destroy_node()

    rclpy.shutdown()

    sys.exit(exit_code)

if __name__ == "__main__":    
    main()

