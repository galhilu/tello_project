import socket

ID=0        #bat_num-1
my_ip="192.168.2.1"+(str(ID))
PIE_PORT=12345
TELLO_ADDR=('192.168.10.1',8889)

welcom_sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
welcom_sock.bind((my_ip,PIE_PORT))
welcom_sock.listen(1)
control_socket_bs,addr=welcom_sock.accept()
control_socket_tello=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
while(True):
    buff=control_socket_bs.recv(20)     #rarely can get more then 1 command at a time
    print(buff)                         #need to implement len a first char
    if buff=="end" or len(buff)==0:
        control_socket_tello.sendto("land".encode('utf-8'),TELLO_ADDR)
        control_socket_tello.sendto("streamoff".encode('utf-8'),TELLO_ADDR)
        break
    else:
        control_socket_tello.sendto(buff,TELLO_ADDR)
print("pie is done")
control_socket_bs.close()
control_socket_tello.close()


