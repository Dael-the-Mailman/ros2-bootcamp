import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

from dataclasses import dataclass
from typing import List

import time
'''
Create a script that has the turtlebot follow an Archimedean spiral
https://en.wikipedia.org/wiki/Archimedean_spiral

'''

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

        # Proportional controller constants
        self.linear_kP = 1.0
        self.angular_kP = 6.0

        # Set limits on the speed of the turtle
        self.MAX_LINEAR_VELOCITY = 1.5
        self.MAX_ANGULAR_VELOCITY = 2.84

        # Set look ahead distance for pure pursuit controller
        self.LOOK_AHEAD_DISTANCE = 0.1

        # Create path for turtlebot to follow
        self.path = self.generate_path()

        # Keep track of lastFoundIndex of turtlebot
        self.lastFoundIndex = 0

        self.index = 0

        self.dt = 0.1

        self.max_dist = 5.0
    
    def distance_to_goal(self, goal_pose):
        return np.sqrt((goal_pose.x - self.odom_pose.x)**2 + \
                       (goal_pose.y - self.odom_pose.y)**2)

    def point_2_point_distance(self, pt1, pt2):
        return np.sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)

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
    
    # Custom sign function to avoid a multiplication by zero
    def sgn(self, num):
        if num >= 0:
            return 1
        else:
            return -1

    def generate_path(self, 
        num_iterations: int = 200,
        radius: float = 4, 
        xOrigin: float = 0.0,
        yOrigin: float = 0.0,
        alpha: float = 5.0,
        beta: float = 1.0,
        dt: float = 0.05) -> List[List[float]]:

        # Initialize x and y coordinates for spiral path
        x = xOrigin
        y = yOrigin

        # Initialize empty path
        path = [[x,y]]
        
        for i in range(num_iterations):
            t: float = i*dt*np.pi
            dx = (alpha*t + beta) * np.cos(t)
            dy = (alpha*t + beta) * np.sin(t)
            path.append([x+dx, y+dy])
        
        return path
    
    def pure_pursuit_controller(self, 
        path: List[List[float]], 
        pose2D: Pose2D, 
        lookAheadDis: float = 0.8, 
        LFindex: int = 0):
        
        # Extract x, y, and theta (aka heading)
        currentX = pose2D.x
        currentY = pose2D.y
        currentHeading = pose2D.theta

        goalPt = path[LFindex + 1]

        # Variables to make loop searching through intersections more efficient
        lastFoundIndex = LFindex
        intersectFound = False
        startingIndex = lastFoundIndex

        for i in range(startingIndex, len(path)-1):
            # Beginning of the line-circle intersection code
            x1 = path[i][0] - currentX
            y1 = path[i][1] - currentY
            x2 = path[i+1][0] - currentX
            y2 = path[i+1][1] - currentY
            dx = x2 - x1
            dy = y2 - y1
            dr = np.sqrt(dx**2 + dy**2)
            D = x1*y2 - x2*y1
            discriminant = (lookAheadDis**2) * (dr**2) - D**2

            if discriminant >= 0:
                sol_x1 = (D * dy + self.sgn(dy) * dx * np.sqrt(discriminant)) / dr**2
                sol_y1 = (D * dy - self.sgn(dy) * dx * np.sqrt(discriminant)) / dr**2
                sol_x2 = (-D * dy + np.abs(dy) * np.sqrt(discriminant)) / dr**2
                sol_y2 = (-D * dy - np.abs(dy) * np.sqrt(discriminant)) / dr**2

                sol_pt1 = [sol_x1 + currentX, sol_y1 + currentY]
                sol_pt2 = [sol_x2 + currentX, sol_y2 + currentY]

                minX = np.fmin(path[i][0], path[i+1][0])
                minY = np.fmin(path[i][1], path[i+1][1])
                maxX = np.fmax(path[i][0], path[i+1][0])
                maxY = np.fmax(path[i][1], path[i+1][1])

                # If one or both of the solutions are in range
                if ((minX <= sol_pt1[0] <= maxX) and (minY <= sol_pt1[1] <= maxY)) or ((minX <= sol_pt2[0] <= maxX) and (minY <= sol_pt2[1] <= maxY)):
                    foundIntersection = True

                    # If both solutions are in range, check which one is better
                    if ((minX <= sol_pt1[0] <= maxX) and (minY <= sol_pt1[1] <= maxY)) and ((minX <= sol_pt2[0] <= maxX) and (minY <= sol_pt2[1] <= maxY)):
                        if self.point_2_point_distance(sol_pt1, path[i+1]) < self.point_2_point_distance(sol_pt2, path[i+1]):
                            goalPt = sol_pt1
                        else:
                            goalPt = sol_pt2

                    # If not both solutions are in range, take the one that's in range
                    else:
                        if (minX <= sol_pt1[0] <= maxX) and (minY <= sol_pt1[1] <= maxY):
                            goalPt = sol_pt1
                        else:
                            goalPt = sol_pt2
                
                    # only exit loop if the solution pt found is closer to the next pt in path than the current pos
                    if self.point_2_point_distance(goalPt, path[i+1]) < self.point_2_point_distance(goalPt, path[i+1]):
                        # update lastFoundIndex and exit
                        lastFoundIndex = i
                        break
                    else:
                        # in case for some reason the robot cannot find intersection in the next path segment, but we also don't want it to go backward
                        lastFoundIndex = i+1
                
                # if no solutions are in range
                else:
                    foundIntersection = False
                    goalPt = [path[lastFoundIndex][0], path[lastFoundIndex][1]]

        absTargetAngle = np.arctan2(goalPt[1]-currentY, goalPt[0]-currentX)
        if absTargetAngle < 0: absTargetAngle += 2*np.pi

        raise NotImplementedError("Still need to figure out why the algorithm is not tracking the path correctly")
        

    def timer_callback(self):
        msg = Twist()

        # First find goal position and heading with pure_pursuit_controller
        # goalPt, lastFoundIndex, absTargetAngle = self.pure_pursuit_controller(self.path, 
        #                                                 self.odom_pose,
        #                                                 lookAheadDis=self.LOOK_AHEAD_DISTANCE,
        #                                                 LFindex=self.lastFoundIndex)
        
        # self.get_logger().info(", ".join(str(x) for x in self.path[self.index]))
        # self.get_logger().info(str(absTargetAngle))

        # Update lastFoundIndex
        # self.lastFoundIndex = lastFoundIndex

        # Temporary fix
        linear_velocity = 0.0
        velocity = 0.1
        angular_velocity = 2.0
        dist_from_center = self.distance_to_goal(Pose2D())

        t = self.index*self.dt

        if(dist_from_center >= self.max_dist):
            linear_velocity = 0.0
            angular_velocity = 0.0
        else:
            linear_velocity = np.sqrt(velocity**2 + ((velocity*t)**2)*(angular_velocity**2))

        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity

        # Add checks to the speed of the bot
        # msg.linear.x = self.clamp(msg.linear.x, self.MAX_LINEAR_VELOCITY)
        # msg.angular.z = self.clamp(msg.angular.z, self.MAX_ANGULAR_VELOCITY)

        self.publisher.publish(msg)

        self.index += 1


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