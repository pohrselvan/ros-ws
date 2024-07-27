import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pohrselvan/ros2_ws/src/install/my_tf2_broadcaster'
