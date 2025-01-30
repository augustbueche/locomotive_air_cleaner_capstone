import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/august/workspace/locomotive_air_cleaner_capstone/src/air_cleaning_pkg/install/air_cleaning_pkg'
