import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RandomTurtle(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 4*np.random.rand(1)[0] - 2 # Random x-velocity
        msg.linear.y = 4*np.random.rand(1)[0] - 2 # Random y-velocity
        msg.angular.z = 2*np.pi*np.random.rand(1)[0] - np.pi # Random angular velocity
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    random_turtle = RandomTurtle()

    rclpy.spin(random_turtle)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()