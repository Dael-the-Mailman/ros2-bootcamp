# ROS 2 Bootcamp

## Table of Contents
1. [Week 1+2](#week-12)
2. [Week 3](#week-3)
3. [Week 4](#week-4)
4. [Week 5](#week-5)

## Week 1+2
### 7 Dimensional State to 6 Dimensional State Pub/Sub
#### Publisher Output
![Publisher Output](/images/6_dim_publisher.png)

#### Subscriber Output
![Subscriber Output](/images/6_dim_subscriber.png)

#### Implementation
I created two files, `publisher_member_function.py` and `subscriber_member_function.py`. The publisher file generated seven random numbers using `numpy` and converted them into a single string delimited by spaces. The subscriber file then receives the string and parses them based on the spaces. The subscriber file then removes the last element and converts the list of numbers into string delimited by commas.

#### Code Referenced
- [`publisher_member_function.py`](/Week%201+2/src/6_dim_state/6_dim_state/publisher_member_function.py)
- [`subscriber_member_function.py`](/Week%201+2/src/6_dim_state/6_dim_state/subscriber_member_function.py)

### Random Goal
![Random Walk](/images/Random%20Walk.png)

#### Implementation
Every 10 seconds, `numpy` would generate three random numbers $\in[0,1)$. It would then scale two of the numbers s.t. $\in[3.4,7.4)$. These two numbers would then be the x and y coordinates of the goal. The third number is scaled s.t. $\in[-\pi,\pi)$ which would correspond with the goal orientation.

To move the turtle towards the goal, I first calculate the distance between the current position of the turtle and the goal position of the turtle. I then calculate the heading of the turtle using $\arctan(\Delta y, \Delta x)$. I then controlled the linear and angular velocity of the turtle using a proportional controller. Once the turtle has reached the goal position, I used the proportional controller to change the angle until the goal angle has been reached. 

#### Code Referenced
- [`random_walk.py`](/Week%201+2/src/random_turtle/random_turtle/random_walk.py)

### Vacuum Cleaning Script
![Vacuum Turtle](/images/Vacuum%20Turtle.png)

#### Implementation
To make the spiral I followed the equation for the Archimedean spiral. The equations for which are given below

$$v_x=v\cos\omega t-\omega(vt+c)\sin\omega t$$

$$v_y=v\sin\omega t+\omega(vt+c)\cos\omega t$$

where $v_x$ is the velocity in the x-direction, $v_y$ is the velocity in the y-direction, $\omega$ is a constant angular velocity, $t$ is the time that has elapsed, and $c$ is the distance between the turtle and the origin.

To stop the turtle from going off the screen, I set a maximum distance of 4 units away from the origin. Once the turtle has reached 4 units away from the origin, a zero velocity signal is sent.

#### Code Referenced
- [`spiral_turtle.py`](/Week%201+2/src/vacuum_turtle/vacuum_turtle/spiral_turtle.py)

## Week 3
### Turtlebot Teleop Control
![Turtle Bot Video](./videos/2024-07-28%2009-56-27.mov)

### Turtlebot Random Goal
![Turtlebot Random Goal Video](./videos/2024-08-11%2016-26-46.mov)

#### Implementation
I took a similar approach to the turtlesim. I first picked a random goal every 10 seconds. Next, I used a proportional controller to manuever the robot to the random point. Finally, I used a separate proportional controller to make the turtlebot face in a random direction. Note that unlike the turtlesim, I had to limit the speed of the turtlebot so that it didn't go too fast. I initially thought there was a built in speed limiter, but I was soon proven wrong.

#### Code Referenced
- [`random_goal.py`](/Week%203/src/turtlebot_random/turtlebot_random/random_goal.py) 

### Turtlebot Vacuum Cleaning
![Turtlebot Vacuum Cleaning Video](/videos/2024-08-11%2016-29-48.mov)

#### Implementation
I implemented a solution very similar to Week 1+2's vacuum cleaning script where you can assume the robot can reach any arbitrary speed you want. The only main difference is that instead of having independent x and y velocity components, the turtlebot will have a single linear velocity that is determined by the equation

$$v_0=\sqrt{v^2+\omega^2(vt+c)}$$

I first tried to implement a pure pursuit controller. I would generate a spiral path and then have the turtlebot follow that path. I spent a good amount of effort on trying to make the pure pursuit controller work but I haven't gotten it to work yet. I would definitely tackle the pure pursuit controller again once I have time.

#### Code Referenced
- [`vacuum.py`](/Week%203/src/turtlebot_vacuum/turtlebot_vacuum/vacuum.py)


## Week 4
TODO

## Week 5
TODO