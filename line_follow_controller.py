import rclpy
import scipy.signal
import time
from rclpy.node import Node
from std_msgs.msg import Float32, String
import numpy as np
import time
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from geometry_msgs.msg import Twist

class followerControl(Node):
    def __init__(self):
        super().__init__('PID_CONTROLLER_NODE_PIERREBORRACHO_APAGNAN_FER_PD')

        qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.pubCompVal = self.create_publisher(Float32, '/compensationVal', 10)
        self.offsetSub = self.create_subscription(Float32, '/offset', self.offset_callback, 10)
        self.encL = self.create_subscription(Float32, '/VelocityEncL', self.encoderL_callback, qos_profile)
        self.encR = self.create_subscription(Float32, '/VelocityEncR', self.encoderR_callback, qos_profile)
        self.pannels = self.create_subscription(String, '/objectName', self.pannel_callback, 10)

        self.get_logger().info('Node initializer')

        self.get_logger().info('line followedsdads controller node initialized')
        self.timer_period = 0.1
        self.radius = 0.05
        self.wheelbase = 0.175
        print("aqui no MAMA")
        self.offset = Float32()
        self.offset = 0.0
        self.velL = 0.0
        self.velR = 0.0
        self.derivative = 0.0
        self.integral = 0.0
        self.proportional = 0.0
        self.lastError = 0.0
        self.compensation = 0.0
        self.kp = 11
        self.ki = 3
        self.kd = 11

        self.pannel_name = "none"
        self.old_pannel_name = "none"


    def semaforo(self):
        twist_msg = Twist()
        if self.pannel_name == "Red-sema":
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = 0.0

        elif self.pannel_name == "Jaune-sema":
            twist_msg.linear.x = 0.5 * 0.05
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = 0.5 * np.clip(self.compensation, -0.05, 0.05)

        else:
            twist_msg.linear.x = 0.05
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = np.clip(self.compensation, -0.05, 0.05)
        return twist_msg

    def offset_callback(self, offset_mm):
        self.offset = offset_mm.data
        twist_msg = Twist()
        self.integral = self.offset * self.timer_period
        self.derivative = (self.offset - self.lastError)/self.timer_period
        self.compensation = self.kp*self.offset + self.ki*self.integral + self.kd*self.derivative
        self.lastError = self.offset
        twist_msg=self.semaforo()
        if not(self.pannel_name == "Jaune-sema") and not(self.pannel_name == "Red-sema") :
            if ((self.pannel_name == "Stop-pannels") and (self.old_pannel_name != "Stop_pannels")):
                twist_msg.linear.x = 0.0
                twist_msg.linear.y = 0.0
                twist_msg.linear.z = 0.0
                twist_msg.angular.x = 0.0
                twist_msg.angular.y = 0.0
                twist_msg.angular.z = 0.0
                time.sleep(3)
            elif self.pannel_name == "Travaux-pannel":
                twist_msg.linear.x = 0.5 * 0.05
                twist_msg.linear.y = 0.0
                twist_msg.linear.z = 0.0
                twist_msg.angular.x = 0.0
                twist_msg.angular.y = 0.0
                twist_msg.angular.z = 0.5 * np.clip(self.compensation, -0.05, 0.05)
            elif self.pannel_name == "none" or ((self.pannel_name == "Stop-pannels") and (self.old_pannel_name == "Stop_pannels")):
                twist_msg.linear.x = 0.05
                twist_msg.linear.y = 0.0
                twist_msg.linear.z = 0.0
                twist_msg.angular.x = 0.0
                twist_msg.angular.y = 0.0
                twist_msg.angular.z = np.clip(self.compensation, -0.05, 0.05)



        self.pubCompVal.publish(Float32(data=self.compensation))
        self.publisher.publish(twist_msg)
        print(self.offset)

    def encoderL_callback(self, msg_cmdL):
        self.velL = msg_cmdL.data


    def encoderR_callback(self, msg_cmdR):
        self.velR = msg_cmdR.data


    def pannel_callback(self, panneau):
        self.old_pannel_name = self.pannel_name
        self.pannel_name = panneau.data


def main(args=None):
    rclpy.init(args=args)
    follow = followerControl()
    rclpy.spin(follow)
    follow.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
