import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/shared-folder/ManchesterChallenges/ros2_ws/install/signal_processing'
