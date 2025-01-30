import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/august/workspace/locomotive_air_cleaner_capstone/install/hello_august'
