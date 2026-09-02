import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ubuntu/robocon/catchrobo_test/ros2_ws/install/catchrobo_single_dxl_test'
