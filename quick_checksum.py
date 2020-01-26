#packet = b'\x124\x00\x0e\x00\x00\x00\x01\x02srover6\x00\xda'
packet = b'\x124\x00\x0e\x00\x00\x00\x01\x02srover6\x02\xda'

def get_checksum(b: bytes):
    checksum = 0
    for val in b:
        checksum += val
    checksum &= 0xffff
    return checksum

def check(packet):
    assert packet[0:2] == b'\x12\x34'
    calc_checksum = get_checksum(packet[4:-2])
    recv_checksum = int.from_bytes(packet[-2:], "big")
    assert calc_checksum == recv_checksum, "%s != %s" % (calc_checksum, recv_checksum)

check(packet)
