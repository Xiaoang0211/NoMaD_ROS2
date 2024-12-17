import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

# Configuration file paths
CONFIG_PATH = "../config/robot.yaml"
JOY_CONFIG_PATH = "../config/joystick.yaml"

class JoyToLocobot(Node):
    def __init__(self):
        super().__init__("Joy2Locobot")
        
        # Load robot configuration
        with open(CONFIG_PATH, "r") as f:
            robot_config = yaml.safe_load(f)
        self.VEL_TOPIC = robot_config["vel_teleop_topic"]

        # Load joystick configuration
        with open(JOY_CONFIG_PATH, "r") as f:
            joy_config = yaml.safe_load(f)
        self.DEADMAN_SWITCH = joy_config["deadman_switch"]
        self.LIN_VEL_BUTTON = joy_config["lin_vel_button"]
        self.ANG_VEL_BUTTON = joy_config["ang_vel_button"]

        self.MAX_V = 0.4
        self.MAX_W = 0.8
        self.RATE = 1.0 / 9.0  # In seconds

        # Initialize variables
        self.vel_msg = Twist()
        self.bumper = False
        self.button = None

        # Publishers
        self.vel_pub = self.create_publisher(Twist, self.VEL_TOPIC, 1)
        self.bumper_pub = self.create_publisher(Bool, "joy_bumper", 1)

        # Subscriber
        self.joy_sub = self.create_subscription(Joy, "joy", self.callback_joy, 1)

        # Timer for publishing
        self.timer = self.create_timer(self.RATE, self.publish_messages)

        self.get_logger().info("Node initialized. Waiting for joystick input...")

    def callback_joy(self, data: Joy):
        """Callback function for the joystick subscriber."""
        self.button = data.buttons[self.DEADMAN_SWITCH]
        bumper_button = data.buttons[self.DEADMAN_SWITCH - 1]

        if self.button is not None:  # Dead-man switch held down
            self.vel_msg.linear.x = self.MAX_V * data.axes[self.LIN_VEL_BUTTON]
            self.vel_msg.angular.z = self.MAX_W * data.axes[self.ANG_VEL_BUTTON]
        else:
            self.vel_msg = Twist()  # Stop the robot
            self.vel_pub.publish(self.vel_msg)

        if bumper_button is not None:
            self.bumper = bool(data.buttons[self.DEADMAN_SWITCH - 1])
        else:
            self.bumper = False

    def publish_messages(self):
        """Publishes velocity and bumper messages at a regular rate."""
        if self.button:
            self.get_logger().info(f"Teleoperating the robot:\n {self.vel_msg}")
            self.vel_pub.publish(self.vel_msg)
        
        bumper_msg = Bool()
        bumper_msg.data = self.bumper
        self.bumper_pub.publish(bumper_msg)

        if self.bumper:
            self.get_logger().info("Bumper pressed!")

def main(args=None):
    rclpy.init(args=args)
    node = JoyToLocobot()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
