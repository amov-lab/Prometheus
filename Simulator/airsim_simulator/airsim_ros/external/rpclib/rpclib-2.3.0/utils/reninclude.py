#! python3

# This is a terrible script that changes the include paths
# in shipped dependencies.

import fileinput
import glob2
import re

files = []
types = ['.cpp', '.cc', '.h', '.hpp', '.hh', '.ipp', '.inl']
for t in types:
    files.extend(glob2.glob('include/**/*' + t))

for f in files:
    print('Processing', f)
    with fileinput.FileInput(f, inplace=True) as fi:
        for line in fi:
            line = re.sub('#(\s*)include (<|")(msgpack)', '#\\1include \\2rpc/\\3', line)
            print(line, end='')

print('done')
