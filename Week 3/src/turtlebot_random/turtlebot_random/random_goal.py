import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

from dataclasses import dataclass

import time

# Create custom dataclass to handle 
# (x,y,theta) coordinates easily
@dataclass
class Pose2D:
    x: float = 0
    y: float = 0
    theta: float = 0

class TurtleBotRandom(Node):
    def __init__(self):
        super().__init__('turtlebot_random')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribe to the pose to see current position of the turtle
        self.subscriber = self.create_subscription(Odometry, 
            '/odom', 
            self.update_pose, 
            10)

        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        ## Assume origin is at (0.0, 0.0)
        self.origin_x = 0.0
        self.origin_y = 0.0

        # Get pose from the /odom topic
        self.odom_pose = Pose2D()

        # Create a goal pose for the turtlebot to go to
        self.goal_pose = Pose2D()

        # Scaling constants for goal position
        self.linear_scale = 8.0
        self.linear_offset = -4.0

        # Position tolerance
        self.tol = 0.1

        # Proportional controller constants
        self.linear_kP = 1.0
        self.angular_kP = 6.0

        self.time_step = 0.0

        # Set limits on the speed of the turtle
        self.MAX_LINEAR_VELOCITY = 1.5
        self.MAX_ANGULAR_VELOCITY = 2.84
    
    def distance(self, goal_pose):
        return np.sqrt((goal_pose.x - self.odom_pose.x)**2 + \
                       (goal_pose.y - self.odom_pose.y)**2)

    def heading(self, goal_pose):
        return np.arctan2(goal_pose.y - self.odom_pose.y, goal_pose.x - self.odom_pose.x)

    def clamp(self, value, max_value):
        if(value > max_value):
            return max_value
        elif(value < -max_value):
            return -max_value
        else:
            return value

    def _quaternion_to_euler(self, pose):
        x = pose.orientation.x
        y = pose.orientation.y
        z = pose.orientation.z
        w = pose.orientation.w
        
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = np.arcsin(t2)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)
        return [yaw, pitch, roll]

    def update_pose(self, data):
        self.odom_pose = Pose2D()
        self.odom_pose.x = data.pose.pose.position.x
        self.odom_pose.y = data.pose.pose.position.y
        self.odom_pose.theta = self._quaternion_to_euler(data.pose.pose)[0]

    def timer_callback(self):
        msg = Twist()

        if (self.time_step == 0.0):
            random_pos = np.random.rand(3)
            self.goal_pose.x = round(self.linear_scale*random_pos[0] + self.linear_offset + self.origin_x, 2)
            self.goal_pose.y = round(self.linear_scale*random_pos[1] + self.linear_offset + self.origin_y, 2)
            self.goal_pose.theta = round(2*np.pi*random_pos[2] - np.pi, 2)
        
        # First reach the goal in the x,y coordinates
        if (self.distance(self.goal_pose) >= self.tol):
            msg.linear.x = self.linear_kP*self.distance(self.goal_pose)
            msg.linear.y = 0.0
            msg.angular.z = self.angular_kP*(self.heading(self.goal_pose) - self.odom_pose.theta)

        # Once goal has been reached in x,y -> go to goal angle
        elif np.abs(self.goal_pose.theta - self.odom_pose.theta) >= self.tol:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = self.angular_kP*(self.goal_pose.theta - self.odom_pose.theta)

        else:
            # If everything is done, publish zero linear and angular velocity
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0

        # Add checks to the speed of the bot
        msg.linear.x = self.clamp(msg.linear.x, self.MAX_LINEAR_VELOCITY)
        msg.linear.y = self.clamp(msg.linear.y, self.MAX_ANGULAR_VELOCITY)
        msg.angular.z = self.clamp(msg.angular.z, self.MAX_ANGULAR_VELOCITY)

        self.publisher.publish(msg)
        
        self.time_step += self.timer_period

        if (self.time_step >= 10.0):
            self.time_step = 0.0


def main(args=None):
    rclpy.init(args=args)

    turtlebot_random = TurtleBotRandom()

    rclpy.spin(turtlebot_random)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtlebot_random.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()