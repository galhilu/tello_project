import socket
import time
import subprocess   

LEADER_IP='192.168.2.10'
MY_IP='192.168.2.11'
BS_IP='192.168.2.1'
PIE_PORT=65432

# sender=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# sender.bind((MY_IP,PIE_PORT))
# sender.connect(BS_IP,PIE_PORT)
# print ("made connection with the base station")
# sender.setblocking(False)
# sniff=True
# while(sniff):
#     ret=subprocess.check_output(['iperf3 -c '+LEADER_IP+' -b 1M -P 1 -t 1| grep "receiver"'],shell=True,text=True)


x=subprocess.check_output(['iperf3 -c 127.0.0.1 -f k -n 100000 -l 512 -P 1| grep "receiver"'],shell=True,text=True)
#x=x[38:42]
print(x)


