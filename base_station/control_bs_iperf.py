from djitellopy import Tello
import cv2
import pygame
import numpy as np
import time
import socket
import av
import os
import ffmpeg
from scapy.all import*

#control V7 video stream integrated 
#------------------------------------------------------
#constants
# Speed of the drone
S = 60
# Frames per second of the pygame window display

FPS = 120

BASE_IP='192.168.10.'
BS_IP='192.168.2.1'
PIE_PORT=12345
#STREAM_PORT=1234


#scale factors index
LEFT_TO_RIGHT=0
RIGHT_TO_LEFT=1
FOR_TO_BACK=2
BACK_TO_FOR=3
#command types:
INSTANT=-2
IDLE=-1
FORWARD_BACKWARD=0
LEFT_RIGHT=1
YAW=2
ELEVATION=3

#ref time  
GLOBAL_REF_TIME=time.time_ns()
#------------------------------------------------------
#global vars



#------------------------------------------------------

class swarm(object):
    def __init__(self,drone_num,stream=False):
        self.drones=[]                                          #list of all drone obj (init of drones in drones_start())
        self.drone_num=drone_num                                #numer of drones in swarm
        self.idle_drones=drone_num-1                            #numer of drones on standby
        self.drones_start(stream)                                
        null_command=command(-1,-1,-1,-1,-1)
        self.ongoing_comands=[null_command,null_command,null_command,null_command]  #list of moving commands for each moving type (axis) see constants.
        self.command_log=[]                                     #stores command history
        self.log_pos_idx=[-1]                                   #list of indexes for the command_log that represent last executed command of each drone (first is leader). start from -1 because the log start empty and is incresed each new command
        self.active=self.drones[0]                              #leader drone
        self.keep_alive_ref=time.time()
        self.last_net_check=time.time()
        self.mesh_adjusting=False                               #indicates whether the swarm is moving for better signal (call a friend) 
        self.sniff_history=[]
        self.sniff_counter=0
        self.sniff_timer=time.time()
        if self.drone_num>=2:
            self.iperf_sock=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
            self.iperf_sock.connect(('192.168.2.11',65432))
            self.iperf_sock.setblocking(False)
        else:
            print("need atleast 2 drones")
            exit()
        print ("swarm init done")
    
    def   drones_start(self,stream):
        for i in range(0, self.drone_num):
            if i==0:
                self.drones.append(Drone(i,stream))
            else:
                self.drones.append(Drone(i,False))
        return
        

    def   drones_stop(self):
        for i in range(0, self.drone_num):
            #self.drones[i].send_command("end")
            self.drones[i].end()
    

    def start_new_command(self,type,val,replay):
        new_command=command(time.time_ns(),replay,type,val,0)
        
        if type==INSTANT:
            if val== "call a friend":           #find and execute swarm handeld operations
                self.call_a_friend()
            elif val== "recall":
                self.recall()
            else:
                self.send_command(new_command)

        elif self.ongoing_comands[type].type==IDLE and type>=0:       #make sure that drone is not already moving on that axis and new command is moving
            self.ongoing_comands[type]=new_command
            self.send_command(new_command)

    def end_new_command(self,type,val):
        if type==INSTANT:
            return
        elif self.ongoing_comands[type].val==val:                   #look at val to drop cases of :key down->,key down <-(is droped in start_new_command()),key up <- (droping this),key up ->
            new_command=command(time.time_ns(),self.ongoing_comands[type].replay,type,0,val)
            self.ongoing_comands[type]=command(-1,-1,-1,-1,-1)
            self.send_command(new_command)
            return
    
    def send_command(self,new_command):
        self.active.command_queue.append(new_command)
        self.command_log.append(new_command)
        self.log_pos_idx[0]=self.log_pos_idx[0]+1

    def keep_alive(self):
        if time.time()-self.keep_alive_ref>5:
            for drone in self.drones:
                drone.keep_alive()
            self.keep_alive_ref=time.time()
            
    def execute(self):
        for drone in self.drones:
            drone.execute()


    def call_a_friend(self):
        if not self.mesh_adjusting:
            if self.idle_drones>0:
                self.log_pos_idx.append(0)
                self.idle_drones=self.idle_drones-1
                self.drones[len(self.drones)-self.idle_drones-1].command_queue.append(command(time.time_ns(),False,INSTANT,"takeoff",0))
            start_pos=self.log_pos_idx.copy()
            for i in range(1,len(self.drones)-self.idle_drones):            #for every non leader active drone
                for j in range(start_pos[i],start_pos[i-1]+1):
                    if self.command_log[j].replay==True:
                        new_command=self.command_log[j]
                        new_command.start_time= new_command.start_time +time_passed()
                        self.drones[i].command_queue.append(new_command)
                        self.log_pos_idx[i]=self.log_pos_idx[i]+1
                self.stop_drone(i)
            self.mesh_adjusting=True
            

    def stop_drone(self,drone_id):                                      #safety command to make sure drone stay in place
        new_command=command(time.time_ns(),False,FORWARD_BACKWARD,0,0)
        self.drones[drone_id].command_queue.append(new_command)
        new_command=command(time.time_ns(),False,LEFT_RIGHT,0,0)
        self.drones[drone_id].command_queue.append(new_command)
        new_command=command(time.time_ns(),False,ELEVATION,0,0)
        self.drones[drone_id].command_queue.append(new_command)
        new_command=command(time.time_ns(),False,YAW,0,0)
        self.drones[drone_id].command_queue.append(new_command)

    def recall(self):
        for i in range(0,len(self.drones)-self.idle_drones):            
            for j in range(self.log_pos_idx[i],0,-1):
                if self.command_log[j].replay==True:
                    new_command=self.command_log[j]
                    new_command.start_time= self.command_log[self.log_pos_idx[i]].start_time - new_command.start_time + time_passed()
                    if new_command.type==FORWARD_BACKWARD or new_command.type==LEFT_RIGHT or new_command.type==ELEVATION or new_command.type==YAW:
                        new_command.val= - new_command.replay_val
                    self.drones[i].command_queue.append(new_command)

            self.drones[i].command_queue.append(command(time.time_ns(),False,INSTANT,"land",0))

        self.idle_drones=self.drone_num-1
        self.log_pos_idx=[-1]
        self.command_log=[]
        self.mesh_adjusting=True

    def get_bat(self):
        self.active.tello.query_battery()

    def mesh_check(self):
        mssg=self.iperf_sock.recv(20)
        if mssg =="adjust" and not self.mesh_adjusting:
            print("want to adjust")
            self.call_a_friend()
        elif self.mesh_adjusting:
            flag=0
            for i in range(1,len(self.drones)):
                if len(self.drones[i].command_queue)>0:
                    flag=1

            if flag==0:
                self.mesh_adjusting=False

        return
            



class Drone(object):

    def __init__(self,id,stream=False):
        print("drone init")
        pi_ip="192.168.2.1"+(str(id))
        self.control_socket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.control_socket.bind((BS_IP,PIE_PORT+id))
        print("bind done")
        self.control_socket.connect((pi_ip,PIE_PORT))
        print("connected")
        self.command_queue=[]
        self.start_time=time.time_ns()
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = 10
        #self.in_air = False
        print("drone init")
        self.send_command("command")
        if stream:
            self.send_command("streamon")
            time.sleep(0.3)
            self.send_command('setfps middle')
            time.sleep(0.3)
        self.send_command("speed {}".format(self.speed))
        
    def end(self):
        self.send_command("end")
        self.control_socket.close()


    def send_command(self,command):
        self.control_socket.send(command.encode('utf-8'))

    def execute(self):
        if len(self.command_queue)!=0:
            if time_passed()> self.command_queue[0].start_time:
                command=self.command_queue.pop(0)
                type=command.type
                if type==INSTANT:
                    val=command.val
                    if val== "takeoff":           
                        self.send_command("takeoff")
                    elif val== "land":
                        self.send_command("land")
                    else:
                        print ("ERORR drone got unknown INSTANT type command")

                elif type==FORWARD_BACKWARD or type==LEFT_RIGHT or type==ELEVATION or type==YAW:
                    self.update(command)

                elif type==IDLE:
                    print("ERORR drone got idle command")

                else:
                    print ("ERORR drone got invalid command type")

    def update(self,command): #format: "rc left_right forward_backward up_down yaw_"
        if command.type==FORWARD_BACKWARD:
            self.send_command('rc {} {} {} {}'.format(self.left_right_velocity, command.val,self.up_down_velocity, self.yaw_velocity))
            self.for_back_velocity=command.val
        elif command.type==LEFT_RIGHT:
            self.send_command('rc {} {} {} {}'.format(command.val, self.for_back_velocity,self.up_down_velocity, self.yaw_velocity))
            self.left_right_velocity=command.val
        elif command.type==ELEVATION:
            self.send_command('rc {} {} {} {}'.format(self.left_right_velocity, self.for_back_velocity,command.val, self.yaw_velocity))
            self.up_down_velocity=command.val
        elif command.type==YAW:
            self.send_command('rc {} {} {} {}'.format(self.left_right_velocity, self.for_back_velocity,self.up_down_velocity, command.val))
            self.yaw_velocity=command.val

    def keep_alive(self):
        self.send_command("keepalive")

            
    # def land(self):
    #     if self.in_air==True:
    #         self.tello.land()          
    #         self.in_air = False
    
    
    # def takeoff(self):
    #     if self.in_air==False:
    #         self.tello.takeoff()
    #         #self.tello.send_command_without_return("takeoff")       #bypass the api because it causes big unnessesery delay 
    #         self.in_air = True

    def is_idle(self):              #list type used as bool returns True if it's not empty
        if self.command_queue:
            return False
        else:
            return True




class command(object):
    def __init__(self,start_time,replay,type,val,replay_val):
        self.start_time=start_time-GLOBAL_REF_TIME      #time of execution
        self.replay=replay                              #should be replayed or not
        self.type=type                                  #command type, see constants in start of file
        self.val=val                                    #value, usually move speed
        self.replay_val=replay_val                        #is used to save original move speed of stopping commaneds for replay


def keydown(key,speed=S):   #need to move takeoff and land to here

    val=speed
    if key == pygame.K_UP or key == pygame.K_DOWN:  # set forward/backword velocity
        type=FORWARD_BACKWARD
        if key == pygame.K_DOWN:
            val=-speed
        reply=True

    elif key == pygame.K_LEFT or key == pygame.K_RIGHT:  # set left/right velocity
        type=LEFT_RIGHT
        if key == pygame.K_LEFT:
            val=-speed
        reply=True

    elif key == pygame.K_w or key == pygame.K_s:  # set up/down velocity
        type=ELEVATION
        if key == pygame.K_s:
            val=-speed
        reply=True

    elif key == pygame.K_a or key == pygame.K_d:  # set yaw counter clockwise/ clockwise velocity
        type=YAW
        if key == pygame.K_a:
            val=-speed
        reply=True

    elif key == pygame.K_t:  # takeoff
        val="takeoff"
        type=INSTANT
        reply=False

    elif key == pygame.K_l:  # land
        val="land"
        type=INSTANT
        reply=False

    elif key == pygame.K_r:  #recall back to base station
        val="recall"
        type=INSTANT
        reply=False

    elif key == pygame.K_f:
        val="call a friend"
        type=INSTANT
        reply=False
    
    else:
        print("ERROR unknown key down:")
        return [-1]

    return [type,val,reply]

def keyup(key):

    val=S
    if key == pygame.K_UP or key == pygame.K_DOWN:  # set forward/backword velocity
        type=FORWARD_BACKWARD
        if key == pygame.K_DOWN:
            val=-S

    elif key == pygame.K_LEFT or key == pygame.K_RIGHT:  # set left/right velocity
        type=LEFT_RIGHT
        if key == pygame.K_LEFT:
            val=-S

    elif key == pygame.K_w or key == pygame.K_s:  # set up/down velocity
        type=ELEVATION
        if key == pygame.K_s:
            val=-S

    elif key == pygame.K_a or key == pygame.K_d:  # set yaw counter clockwise/ clockwise velocity
        type=YAW
        if key == pygame.K_a:
            val=-S

    elif key == pygame.K_t   or key == pygame.K_l or key == pygame.K_r or key == pygame.K_f :            #instant commands are handeled on key down so ignore
        type=INSTANT
        val=-1
    
    else:
        print("ERROR unknown key up:")
        return [-1]       


    return [type,val]

def time_passed(ref_point=GLOBAL_REF_TIME):
    return time.time_ns()-ref_point

def main():
    drone_num=2
    stream=True
    drone_swarm=swarm(drone_num,stream)
    
    if stream:
        #os.system("ssh bat1@192.168.2.10 ffmpeg -i udp:192.168.2.10:11111 -r 15 -f rtp rtp:192.168.2.1:11111")
        os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'protocol_whitelist;file,rtp,udp'
        i=0
        cap = cv2.VideoCapture('stream.sdp')
        prev_frame_time=0
        fps_samples=[0,0,0,0,0,0,0,0,0,0]
        #frame_skip=30
        fps_timer=time.time()
        print("video on")

    pygame.init()
    pygame.display.set_caption("Tello control")
    screen = pygame.display.set_mode([960, 720])
    pygame.time.set_timer(pygame.USEREVENT + 1, 1000 // FPS)
    running=True

    while running:

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                return_value=keydown(event.key)
                if len(return_value)==1:        #unknown key
                    print (event.key)
                    continue
                print (f"key down {event.key}") 
                drone_swarm.start_new_command(return_value[0],return_value[1],return_value[2])
            elif event.type == pygame.KEYUP:
                return_value=keyup(event.key)
                if len(return_value)==1:        #unknown key
                    print (event.key)
                    continue        
                print (f"key up {event.key}")
                drone_swarm.end_new_command(return_value[0],return_value[1])
        
        drone_swarm.execute()
        drone_swarm.keep_alive()
        drone_swarm.mesh_check()
        if stream:
            if cap.isOpened() and time.time()-fps_timer>(1 / FPS):
                fps_timer=time.time()
                ret, frame = cap.read()
                if ret:
                    frame_time=time.time()
                    inter_frame_time=frame_time-prev_frame_time
                    fps=1/inter_frame_time
                    prev_frame_time=frame_time
                    fps_samples[i]=fps
                    i+=1
                    if i>len(fps_samples)-1:
                        avg_fps=0
                        for x in fps_samples:
                            avg_fps+=x
                        avg_fps=avg_fps/len(fps_samples)
                        print(avg_fps)
                        print(fps_samples)
                        i=0
                    screen.fill([0,0,0])
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    frame = np.rot90(frame)
                    frame = pygame.surfarray.make_surface(frame)
                    screen.blit(frame, (0,0))
                    #time.sleep(1 / FPS)
                pygame.display.update()

        #-------- frame skip version-----------
        # if stream:
        #     if cap.isOpened():
        #         ret, frame = cap.read()
        #         if ret:
        #             if frame_skip==0:
        #                 frame_time=time.time()
        #                 inter_frame_time=frame_time-prev_frame_time
        #                 fps=1/inter_frame_time
        #                 prev_frame_time=frame_time
        #                 fps_samples[i]=fps
        #                 i+=1
        #                 if i>len(fps_samples)-1:
        #                     avg_fps=0
        #                     for x in fps_samples:
        #                         avg_fps+=x
        #                     avg_fps=avg_fps/len(fps_samples)
        #                     print(avg_fps)
        #                     #print(fps_samples)
        #                     i=0
        #                 screen.fill([0,0,0])
        #                 frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        #                 frame = np.rot90(frame)
        #                 #cv2.putText(frame, f'FPS: {fps:.2f}%', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        #                 frame = pygame.surfarray.make_surface(frame)
        #                 screen.blit(frame, (0,0))
        #                 #time.sleep(1 / FPS)
        #             else:
        #                 frame_skip-=1
        #                 print("skiped")
        #         else:
        #             frame_skip+=1
        #         pygame.display.update()
        #------------------------------------------

        #screen.fill([0, 0, 0])
        pygame.display.update()
        
        

    # Call it always before finishing. To deallocate resources.
    drone_swarm.drones_stop()
    if stream:
        cap.release()



if __name__ == '__main__':
    main()



#----------running instructions 
# 1. change number of drones at the start of main as desiered
# 2. on each pie run forwarder.py
# 3. if you want streaming, run on the leader: ffmpeg -i udp:192.168.2.10:11111 -r 15 -f rtp rtp:192.168.2.1:11111
#     you can tweak the latency by adding arguments
# 4. start drones and connect pie to Drone
# 5. run the code and make sure that you are in the correct folder (needs to have stream.sdp) 