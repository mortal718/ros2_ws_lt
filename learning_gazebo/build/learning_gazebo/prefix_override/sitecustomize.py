import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hri01/ros_bot_lt/src/learning_gazebo/install/learning_gazebo'
