import socket
import time

MY_ADDR=('192.168.2.11','8889')
TELLO_ADDR=('192.168.10.1','8889')


sender=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sender.bind(MY_ADDR)
while(1):
    time.sleep(3)
    sender.sendto("command".encode('utf-8'),TELLO_ADDR)