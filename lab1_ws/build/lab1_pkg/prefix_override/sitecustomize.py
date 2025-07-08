import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/patchy/arc_ws/ARC-PI-ROSCODE/lab1_ws/install/lab1_pkg'
