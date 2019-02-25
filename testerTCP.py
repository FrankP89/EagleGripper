import socket
import sys
import time
#import Faulhaber_Control
#import Shinano_Control

TCP_IP = '172.31.1.147'
TCP_PORT = 30001
BUFFER_SIZE = 1024
MESSAGE = 'Hello from PI'

s = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
s.connect((TCP_IP,TCP_PORT))


#s.sendto(MESSAGE.encode('utf-8'),(TCP_IP,TCP_PORT))

    
data = s.recv(BUFFER_SIZE)
s.close

print ('received data: ', data)