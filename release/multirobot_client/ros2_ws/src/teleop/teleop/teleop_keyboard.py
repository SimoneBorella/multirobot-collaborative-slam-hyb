import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Teleop Twist Keyboard
---------------------------
Moving around:
        w
   a    s    d
        x

q/z : increase/decrease speed
e/c : increase/decrease turn
space : STOP

CTRL-C to quit
"""

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.declare_parameter('namespace', '')
        ns = self.get_parameter('namespace').get_parameter_value().string_value
        self.cmd_vel_topic = f"/{ns}/cmd_vel" if ns else "/cmd_vel"
        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.speed = 0.2
        self.turn = 1.0

        self.get_logger().info(f"Publishing cmd_vel on: {self.cmd_vel_topic}")
        self.get_logger().info(msg)

    def publish_twist(self, linear, angular):
        t = Twist()
        t.linear.x = linear
        t.angular.z = angular
        self.pub.publish(t)

        print(f"Linear: {linear:.2f}, Angular: {angular:.2f}")

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    global settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    node = TeleopNode()

    linear = 0.0
    angular = 0.0

    try:
        while rclpy.ok():
            key = getKey()
            if key == '\x03':
                break

            if key == "w":
                linear += 0.05
                node.publish_twist(linear, angular)
            elif key == "x":
                linear -= 0.05
                node.publish_twist(linear, angular)
            elif key == "a":
                angular += 0.1
                node.publish_twist(linear, angular)
            elif key == "d":
                angular -= 0.1
                node.publish_twist(linear, angular)
            elif key == "s":
                linear = 0.0
                angular = 0.0
                node.publish_twist(linear, angular)
            elif key == '':
                pass
            else:
                node.get_logger().info("Unknown key")
            
    except Exception as e:
        node.get_logger().error(str(e))
    finally:
        node.publish_twist(0, 0)
        rclpy.shutdown()
