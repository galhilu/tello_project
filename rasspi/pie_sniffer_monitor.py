from scapy.all import*
import socket
import time


MY_IP='192.168.2.11'
BS_IP='192.168.2.1'
PIE_PORT=65432

#conf.use_pcap=True      #from example dont know if needed

# def callBack(pkg):
#     if pkg.haslayer(RadioTap):
#         if pkg.dBm_AntSignal<-55:
#             s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

sender=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sender.bind((MY_IP,PIE_PORT))
sender.connect(BS_IP,PIE_PORT)
print ("made connection with the base station")
sender.setblocking(False)
sniff=True
while True:
    if sniff:
        pkg=sniff(iface='mon1',count=1)         #return list of packet obj
        if pkg[0].haslayer(RadioTap):
            if type(pkg[0].dBm_AntSignal)==int:
                print("dBm="+pkg[0].dBm_AntSignal)           
                if int(pkg[0].dBm_AntSignal)<-55:
                    sender.send(str(pkg[0].dBm_AntSignal).encode('utf-8'))      #tell BS that signal is getting low
                    sniff=False
    try:
        msg=sender.recv(1024)
        if msg.decode('utf-8')=="adjusting done":               #wait for BS to say that adjusting is done and go back to channel sampling
            sniff=True
        elif msg.decode('utf-8')=="adjusting start":
            sniff=False
        else:
            print("got unknown message:"+msg.decode('utf-8'))

    except:
        pass
    
    time.sleep(1)
































while True:
    pkg=sniff(iface='wlan0',count=1)         #return list of packet obj
    print("sniffed")
    if pkg[0].haslayer(RadioTap):
        print("has header!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        if type(pkg[0].dBm_AntSignal)==int:
            print("dBm="+pkg[0].dBm_AntSignal)           
            if int(pkg[0].dBm_AntSignal)<-55:
                print("reception too low")      #tell BS that signal is getting low
    
    time.sleep(0.1)