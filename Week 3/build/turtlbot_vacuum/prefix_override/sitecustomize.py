import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dev-desktop/workspace/ros2-bootcamp/Week 3/install/turtlbot_vacuum'
