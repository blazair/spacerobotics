import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/blazar/workspaces/rockytimes_ws/install/terrain_mapping_drone_control'
