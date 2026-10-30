import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sanket/orca_robotics_task/nav2_goal_task/install/nav2_goal_navigator'
