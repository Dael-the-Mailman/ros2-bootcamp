import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

"""
Create an Archimedean Spiral using TurtleSim
https://en.wikipedia.org/wiki/Archimedean_spiral
"""

class SpiralTurtle(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Subscribe to the pose to see current position of the turtle
        self.subscriber = self.create_subscription(Pose, 
            '/turtle1/pose', 
            self.update_pose, 
            10)

        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Angular velocity is a fixed quantity
        # Set the angular velocity to some arbitrary constant
        # The higher the value the tighter the spiral
        self.angular_vel = 1.0

        # An additional parameter to control how the tight the spiral is
        # Lower the value the tighter the spiral
        self.linear_vel = 0.1

        # Track the time elapsed from starting
        self.time_step = 0

        ## Assume same starting position
        self.origin_x = 5.544445
        self.origin_y = 5.544445

        ## Assume turtle starts in the same position as the origin
        self.pose = Pose()
        self.pose.x = self.origin_x
        self.pose.y = self.origin_y

        # Set the maximum distance from the origin
        self.max_dist = 5.0

    def update_pose(self, data):
        self.pose = data

    def timer_callback(self):
        msg = Twist()

        ## Calculate distance from center
        dist_from_center = np.sqrt((self.origin_x - self.pose.x)**2 + \
                                   (self.origin_y - self.pose.y)**2)

        # If the turtle is at the maximum distance away from the origin
        # Stop the turtle
        if(dist_from_center >= self.max_dist):
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0
            self.publisher.publish(msg)
        else:
            msg.linear.x = self.linear_vel*np.cos(self.angular_vel*self.time_step) - \
                        self.angular_vel*(self.linear_vel*self.time_step+dist_from_center)*np.sin(self.angular_vel*self.time_step)
            msg.linear.y = self.linear_vel*np.sin(self.angular_vel*self.time_step) + \
                        self.angular_vel*(self.linear_vel*self.time_step+dist_from_center)*np.cos(self.angular_vel*self.time_step)
            msg.angular.z = self.angular_vel
            self.publisher.publish(msg)

            self.time_step += self.timer_period

def main(args=None):
    rclpy.init(args=args)

    spiral_turtle = SpiralTurtle()

    rclpy.spin(spiral_turtle)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()