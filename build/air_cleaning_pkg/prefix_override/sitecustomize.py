import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/amhun/workspace/locomotive_air_cleaner_capstone/install/air_cleaning_pkg'
