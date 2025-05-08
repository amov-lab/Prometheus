#! python2

# This is used to generate binary msgpack-rpc calls for the 
# unit tests.

import msgpack

def make_pack(call):
    s = msgpack.packb(call)
    return ''.join(map(lambda c:'\\x%02x'%c, map(ord, s)))

calls = [
    [0, 0, 'dummy_void_zeroarg', []], 
    [0, 0, 'dummy_void_singlearg', [42]],
    [0, 0, 'dummy_void_multiarg', [363, 12]],
    [0, 0, 'dummy_void_singlearg', []],
    [0, 0, 'dummy_void_singlearg', [42, 43]],
]

for c in calls:
    pack = make_pack(c)
    print 'const unsigned char raw_msg[] = "{}";'.format(pack)

