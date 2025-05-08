#! python3

# This is a terrible script that renames the namespaces of
# dependencies that rpclib ships with (in order to
# avoid name collisions during linking)

import fileinput
import glob2
import re

targets = ['asio', 'fmt', 'msgpack'] # msgpack is on the interface

files = []
types = ['.cpp', '.cc', '.h', '.hpp', '.hh', '.ipp', '.inl']
for t in types:
    files.extend(glob2.glob('../dependencies/**/*' + t))
    files.extend(glob2.glob('../include/msgpack/**/*' + t))

for f in files:
    print('Processing', f)
    with fileinput.FileInput(f, inplace=True) as fi:
        for line in fi:
            for t in targets:
                line = line.replace('using namespace ' + t, 'using namespace clmdep_' + t)
                line = line.replace('namespace ' + t, 'namespace clmdep_' + t)
                line = line.replace(t + '::', 'clmdep_' + t + '::')
                line = line.replace('using ' + t, 'using clmdep_' + t)
                line = line.replace('asio_', 'clmdep_asio_')
                line = line.replace('clmdep_clmdep_', 'clmdep_')
            print(line, end='')

usages = { 'asio': 'RPCLIB_ASIO', 'fmt': 'RPCLIB_FMT', 'msgpack': 'RPCLIB_MSGPACK' }

files = []
types = ['.h', '.cc', '.inl']
for t in types:
    files.extend(glob2.glob('../include/**/*' + t))
    files.extend(glob2.glob('../lib/**/*' + t))
    files.extend(glob2.glob('../tests/**/*' + t))

for f in files:
    print('Processing', f)
    with fileinput.FileInput(f, inplace=True) as fi:
        for line in fi:
            for a, b in usages.items():
                line = line.replace('using namespace ' + a, 'using namespace ' + b)
                line = line.replace(a + '::', b + '::')
            print(line, end='')

print('done')
