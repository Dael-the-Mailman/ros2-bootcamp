import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dev-desktop/workspace/ros2-bootcamp/W1/install/random_turtle'
