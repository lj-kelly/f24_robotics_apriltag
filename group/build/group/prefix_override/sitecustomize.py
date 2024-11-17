import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/liam/f24_robotics_apriltag/f24_robotics_apriltag/group/install/group'
