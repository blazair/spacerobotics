import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/blazar/workspaces/cartip_ws/install/cart_pole_optimal_control'
