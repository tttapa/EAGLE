import socket
import struct

ip = "127.0.0.1"
port = 4445

values = range(0,16,1)
bytevalues = struct.pack('f'*len(values), *values)

print([ "0x%02x" % b for b in bytevalues ])

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(bytevalues  , (ip, port))