import time
from ..config import ConfigManager

rover_config = ConfigManager.get_rover_config()


def get_checksum(b: bytes):
    checksum = 0
    for val in b:
        checksum += val
    checksum &= 0xff
    return checksum



class Packet:
    PACKET_CODES = rover_config.packet_codes
    WRITE_COMMANDS = rover_config.write_commands
    WRITE_CODES = rover_config.write_codes

    SEGMENT_DELIMITER = b"\t"
    PACKET_START = b"\x12\x34"
    PACKET_STOP = b"\n"

    CODE_TO_TYPE = {
        'd': int,
        'l': int,
        'f': float,
        's': str,
    }

    WRITE_PACKET_NUM = 0

    def __init__(self):
        self.packet = b""
        self.index = 0
        self.recv_checksum = 0
        self.calc_checksum = 0

        self.timestamp = 0.0

        self.segments = []
        self.packet_num = 0
        self.identifier = ""
        self.code = ""

        self.args = None

        self.type_mapping = {
            'd': int,
            'u': int,
            'f': float,
            's': bytes.decode,
        }

    @classmethod
    def from_bytes(cls, packet: bytes):
        self = cls()
        self.packet = packet
        self.segments = packet[:-2].split(self.SEGMENT_DELIMITER)
        self.packet_num = int(self.segments[0])
        self.identifier = self.segments[1].decode()
        self.code = self.PACKET_CODES[self.identifier]
        self.timestamp = time.time()
        return self

    @classmethod
    def from_args(cls, command_name: str, *args):
        if command_name not in cls.WRITE_COMMANDS:
            raise ValueError("%s is not available write command" % command_name)

        self = cls()
        self.code = cls.WRITE_CODES[command_name]
        self.packet_num = cls.WRITE_PACKET_NUM
        self.identifier = command_name
        self.args = args

        command = cls.WRITE_COMMANDS[command_name].encode()
        assert len(args) == len(cls.WRITE_CODES[command_name]), \
            "%s != %s" % (len(args), len(cls.WRITE_CODES[command_name]))

        packet = cls.PACKET_START
        packet += str(cls.WRITE_PACKET_NUM).encode()
        packet += b"\t" + command
        for index, code in enumerate(self.code):
            assert cls.CODE_TO_TYPE[code] == type(args[index]), "%s != %s" % (args[index], cls.CODE_TO_TYPE[code])
            packet += b"\t" + str(args[index]).encode()

        checksum = get_checksum(packet[2:])
        packet += ("%02x" % checksum).encode('ascii')
        packet += cls.PACKET_STOP

        cls.WRITE_PACKET_NUM += 1

        self.packet = packet
        self.timestamp = time.time()

        return self

    @classmethod
    def from_packet(cls, packet: "Packet"):
        return cls.from_args(packet.identifier, *packet.args)

    def checksum(self) -> bool:
        self.recv_checksum = int(self.packet[-2:], 16)
        self.calc_checksum = get_checksum(self.packet[:-2])
        return self.recv_checksum == self.calc_checksum

    def iter(self):
        for segment_index, segment in enumerate(self.segments[2:]):
            segment_type = self.code[segment_index]
            yield segment_index, self.type_mapping[segment_type](segment)

    def __str__(self):
        return str(self.packet)
    
    def __bool__(self):
        return bool(self.packet)
