import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/blazar/lawny_ws/install/first_order_boustrophedon_navigator'
