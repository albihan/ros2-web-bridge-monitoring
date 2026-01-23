import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/albihan/ros2_web_monitoring/ros2_ws/install/dashboard_bridge'
