import socket
import time
import subprocess   

#LEADER_IP='192.168.2.10'
LEADER_IP='127.0.0.1'
MY_IP='192.168.2.11'
BS_IP='192.168.2.1'
PIE_PORT=65432
SAMPLE_NUM=3
THREASHOLD=0.5  #need to adjust with experiments

welcom_sock=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
welcom_sock.bind((MY_IP,PIE_PORT))
welcom_sock.listen(1)
sender,addr=welcom_sock.accept()
print ("made connection with the base station")
sender.setblocking(False)
msg="adjust"
counter=0
tp_avg=[]
sniff=True
while(sniff):
    ret=subprocess.check_output(['iperf3 -c '+LEADER_IP+' -b 1M -P 1 -t 1| grep "receiver"'],shell=True,text=True)
    #ret=subprocess.check_output(['iperf3 -c '+LEADER_IP+' -f k -n 100000 -l 512 -P 1| grep "receiver"'],shell=True,text=True)
    ret=float(ret[38:42])
    counter+=1
    if len(tp_avg)<SAMPLE_NUM:
        tp_avg.append(ret)
        continue
    tp_avg[counter%len(tp_avg)]=ret
    sum=0
    for samp in tp_avg:
        sum+=samp
    sum=sum/SAMPLE_NUM
    if sum<=THREASHOLD:
        sender.send(msg.encode('utf-8'))


