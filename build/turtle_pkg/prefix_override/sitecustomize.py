import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mahmoud-elzankalony/ros2_ws2/install/turtle_pkg'
