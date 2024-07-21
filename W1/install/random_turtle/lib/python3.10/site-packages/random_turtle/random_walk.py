import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time

class RandomTurtle(Node):
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

        ## Assume same starting position
        self.origin_x = 5.544445
        self.origin_y = 5.544445

        ## Assume turtle starts in the same position as the origin
        self.pose = Pose()
        self.pose.x = self.origin_x
        self.pose.y = self.origin_y
        self.pose.theta = 0.0

        # Scaling constants for goal position
        self.linear_scale = 4.0
        self.linear_offset = -2.0

        # Create initial goal_pose
        self.goal_pose = Pose()

        # Position tolerance
        self.tol = 0.01

        # Proportional controller constants
        self.linear_kP = 2.0
        self.angular_kP = 6.0

        self.time_step = 0.0

    def update_pose(self, data):
        self.pose = data

    def distance(self, goal_pose):
        return np.sqrt((goal_pose.x - self.pose.x)**2 + \
                       (goal_pose.y - self.pose.y)**2)

    def heading(self, goal_pose):
        return np.arctan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def timer_callback(self):
        msg = Twist()

        if (self.time_step > 10.0):
            self.time_step = 0.0

        if (self.time_step == 0.0):
            random_pos = np.random.rand(3)
            self.goal_pose.x = self.linear_scale*random_pos[0] + self.linear_offset + self.origin_x
            self.goal_pose.y = self.linear_scale*random_pos[1] + self.linear_offset + self.origin_y
            self.goal_pose.theta = 2*np.pi*random_pos[2] - np.pi
        
        # First reach the goal in the x,y coordinates
        if (self.distance(self.goal_pose) >= self.tol):
            msg.linear.x = self.linear_kP*self.distance(self.goal_pose)
            msg.linear.y = 0.0
            msg.angular.z = self.angular_kP*(self.heading(self.goal_pose) - self.pose.theta)

            self.publisher.publish(msg)
        
        # Once goal has been reached in x,y -> go to goal angle
        elif np.abs(self.goal_pose.theta - self.pose.theta) >= self.tol:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = self.angular_kP*(self.goal_pose.theta - self.pose.theta)

            self.publisher.publish(msg)
        else:
            # If everything is done, publish zero linear and angular velocity
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0

            self.publisher.publish(msg)
        
        self.time_step += self.timer_period

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