import socket
import sys

UDP_IP = "127.0.0.1"
UDP_PORT = 49152
MESSAGE = "Hello, World!"

# Create a UDP socket
sock = socket.socket(socket.AF_INET, # Internet
        socket.SOCK_DGRAM) # UDP
print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT

try:
    # Send message
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
    print >>sys.stderr, "Sending message: :", MESSAGE

    # Receive response
    print >>sys.stderr, "Waiting to receive"
    data, server = sock.recvfrom(4096)
    print >>sys.stderr, "Received: ", data
finally:
    print >>sys.stderr, "Closing socket"
    sock.close()
