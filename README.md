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
- [`publisher_member_function.py`](/W1/src/6_dim_state/6_dim_state/publisher_member_function.py)
- [`subscriber_member_function.py`](/W1/src/6_dim_state/6_dim_state/subscriber_member_function.py)

### Random Goal
![Random Walk](/images/Random%20Walk.png)

#### Implementation
Every 10 seconds, `numpy` would generate three random numbers $\in[0,1)$. It would then scale two of the numbers s.t. $\in[3.4,7.4)$. These two numbers would then be the x and y coordinates of the goal. The third number is scaled s.t. $\in[-\pi,\pi)$ which would correspond with the goal orientation.

To move the turtle towards the goal, I first calculate the distance between the current position of the turtle and the goal position of the turtle. I then calculate the heading of the turtle using $\arctan(\Delta y, \Delta x)$. I then controlled the linear and angular velocity of the turtle using a proportional controller. Once the turtle has reached the goal position, I used the proportional controller to change the angle until the goal angle has been reached. 

#### Code Referenced
- [`random_walk.py`](/W1/src/random_turtle/random_turtle/random_walk.py)

### Vacuum Cleaning Script
![Vacuum Turtle](/images/Vacuum%20Turtle.png)

#### Implementation
To make the spiral I followed the equation for the Archimedean spiral. The equations for which are given below
$$v_x=v\cos\omega t-\omega(vt+c)\sin\omega t$$
$$v_y=v\sin\omega t+\omega(vt+c)\cos\omega t$$
where $v_x$ is the velocity in the x-direction, $v_y$ is the velocity in the y-direction, $\omega$ is a constant angular velocity, $t$ is the time that has elapsed, and $c$ is the distance between the turtle and the origin.

To stop the turtle from going off the screen, I set a maximum distance of 4 units away from the origin. Once the turtle has reached 4 units away from the origin, a zero velocity signal is sent.

#### Code Referenced
- [`spiral_turtle.py`](/W1/src/vacuum_turtle/vacuum_turtle/spiral_turtle.py)

## Week 3
TODO

## Week 4
TODO

## Week 5
TODO