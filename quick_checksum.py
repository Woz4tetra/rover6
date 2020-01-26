#packet = b'0\tready\t1465622\thana'
packet = b'14629\tready\t175395\thanac'

def get_checksum(b: bytes):
    checksum = 0
    for val in b:
        checksum += val
    checksum &= 0xff
    return checksum

def check(packet):
    calc_checksum = get_checksum(packet[:-2])
    recv_checksum = int(packet[-2:], 16)
    assert calc_checksum == recv_checksum, "%s != %s" % (calc_checksum, recv_checksum)

check(packet)
