import ccsdspy
from ccsdspy import PacketField, PacketArray
from ccsdspy.utils import split_packet_bytes
   

import os
print(os.path.getsize('mypackets.bin'))

pkt = ccsdspy.FixedLength([
    PacketField(name='SHCOARSE', data_type='uint', bit_length=32),
    PacketField(name='SHFINE',   data_type='uint', bit_length=20),
    PacketField(name='OPMODE',   data_type='uint', bit_length=3),
    PacketField(name='SPACER',   data_type='fill', bit_length=1),
    PacketField(name='VOLTAGE',  data_type='int',  bit_length=8),
    PacketArray(
        name='SENSOR_GRID',
        data_type='uint',
        bit_length=16,
        array_shape=(32, 32),
        array_order='C'
    ),
])
    
result = pkt.load('mypackets.bin', include_primary_header=True)
print(result)

print(result['SHCOARSE'])
print(result['SHFINE'])
print(result['OPMODE'])
print(result['VOLTAGE'])
print(result['SENSOR_GRID'])

from ccsdspy.utils import read_primary_headers

header_arrays = read_primary_headers('mypackets.bin')

print(header_arrays)
