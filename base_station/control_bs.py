from djitellopy import Tello
import cv2
import pygame
import numpy as np
import time
import socket
import av
import os

#control V7 video stream integrated 
#------------------------------------------------------
#constants
# Speed of the drone
S = 60
# Frames per second of the pygame window display
# A low number also results in input lag, as input information is processed once per frame.
FPS = 30

BASE_IP='192.168.10.'
BS_IP='192.168.2.1'
PIE_PORT=65432
STREAM_PORT=1234


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
        #self.pie_soc=self.drones_start(stream) 
        #self.pie_soc.setblocking(False)
        self.drones_start(stream)                                
        
        #self.control_soc=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #self.control_soc.bind((BS_IP,Tello.CONTROL_UDP_PORT))
        #self.video_soc=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #self.video_soc.bind((BS_IP,STREAM_PORT))
        null_command=command(-1,-1,-1,-1,-1)
        self.ongoing_comands=[null_command,null_command,null_command,null_command]  #list of moving commands for each moving type (axis) see constants.
        self.command_log=[]                                     #stores command history
        self.log_pos_idx=[-1]                                   #list of indexes for the command_log that represent last executed command of each drone (first is leader). start from -1 because the log start empty and is incresed each new command
        self.active=self.drones[0]                              #leader drone
        self.keep_alive_ref=time.time()
        self.last_net_check=time.time()
        self.mesh_adjusting=False                               #indicates whether the swarm is moving for better signal (call a friend) 
        print ("swarm init done")
    
    def   drones_start(self,stream):
        for i in range(0, self.drone_num):
            if i==0:
                self.drones.append(Drone(BASE_IP +str(i+100),stream))
            else:
                self.drones.append(Drone(BASE_IP +str(i+100),False))
        #temp_listener=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        #temp_listener.bind((BS_IP,PIE_PORT))
        #temp_listener.listen()
        #conn, addr = temp_listener.accept()
        #return conn
        return
        

    def   drones_stop(self):
        for i in range(0, self.drone_num):
            self.drones[i].tello.end()
    

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
            #for drone in self.drones:
            #    drone.keep_alive()
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
            msg="adjusting start"               #tell pie to stop sniffing until mesh adjusted 
            self.pie_soc.send(msg.encode('utf-8'))
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
        msg="adjusting start"               #tell pie to stop sniffing until mesh adjusted 
        self.pie_soc.send(msg.encode('utf-8'))
        self.mesh_adjusting=True

    def get_bat(self):
        self.active.tello.query_battery()

    def mesh_check(self):
        if not self.mesh_adjusting:                 #if mesh is adjusting , pie's not supposed to sample signal 
            try:                                    #try because pie_socke.recv is non blocking and will throw error if no data to read
                mssg=self.pie_soc.recv(1024)        
                print("signal is low, starting mesh adjustment")
                print(mssg.decode('utf-8'))
                inplace=True
                for move in self.ongoing_comands:       #check that leader is not moving to avoid adjusting with incomplete log
                    if move !=command(-1,-1,-1,-1,-1):
                        inplace=False
                        print("stop for friend")
                        break
                if inplace:
                    self.call_a_friend()                   
                return
            except:
                return
            
        else:
            adjusting_drones=self.drone_num-self.idle_drones-1
            done=True
            for i in range(1,adjusting_drones+1):
                if not self.drones[1].is_idle:
                    done=False
                    break
                        #check if adjusting is done in order to tell pie to start sampling agian 
            if done:
                msg="adjusting done"
                self.pie_soc.send(msg.encode('utf-8'))
                self.mesh_adjusting=False
            



class Drone(object):

    def __init__(self,ip,stream=False):
        self.tello = Tello(ip)
        self.command_queue=[]
        self.start_time=time.time_ns()
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = 10
        self.in_air = False
        self.tello.connect(False)
        if stream:
            self.tello.streamon()
            #self.tello.set_video_bitrate(Tello.BITRATE_1MBPS)
            #self.tello.set_video_resolution(Tello.RESOLUTION_480P)
        self.tello.set_speed(self.speed)
        
    def execute(self):
        if len(self.command_queue)!=0:
            if time_passed()> self.command_queue[0].start_time:
                command=self.command_queue.pop(0)
                type=command.type
                if type==INSTANT:
                    val=command.val
                    if val== "takeoff":           
                        self.takeoff()
                    elif val== "land":
                        self.land()
                    else:
                        print ("ERORR drone got unknown INSTANT type command")

                elif type==FORWARD_BACKWARD or type==LEFT_RIGHT or type==ELEVATION or type==YAW:
                    self.update(command)

                elif type==IDLE:
                    print("ERORR drone got idle command")

                else:
                    print ("ERORR drone got invalid command type")

    def update(self,command):
        try:                            #in try & except because it some time rises 
            self.tello.query_battery()      #block untill get answer to up the chances that the command will be recvied
        except:
            pass
        if command.type==FORWARD_BACKWARD:
            self.tello.send_rc_control(self.left_right_velocity, command.val,self.up_down_velocity, self.yaw_velocity)
            self.for_back_velocity=command.val
        elif command.type==LEFT_RIGHT:
            self.tello.send_rc_control(command.val, self.for_back_velocity,self.up_down_velocity, self.yaw_velocity)
            self.left_right_velocity=command.val
        elif command.type==ELEVATION:
            self.tello.send_rc_control(self.left_right_velocity, self.for_back_velocity,command.val, self.yaw_velocity)
            self.up_down_velocity=command.val
        elif command.type==YAW:
            self.tello.send_rc_control(self.left_right_velocity, self.for_back_velocity,self.up_down_velocity, command.val)
            self.yaw_velocity=command.val

    def keep_alive(self):
        print("before")
        self.tello.connect(False)
        print("after")

            
    def land(self):
        if self.in_air==True:
            self.tello.land()          
            self.in_air = False
    

    def takeoff(self):
        if self.in_air==False:
            #self.tello.takeoff()
            self.tello.send_command_without_return("takeoff")       #bypass the api because it causes big unnessesery delay 
            self.in_air = True

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
    drone_num=1
    stream=True
    drone_swarm=swarm(drone_num,stream)
    prev_frame_time=0
    fps_samples=[0,0,0,0,0,0,0,0,0,0]
    
    os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'protocol_whitelist;file,rtp,udp'
    i=0
    cap = cv2.VideoCapture('stream.sdp')
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
        #drone_swarm.mesh_check()
        if stream:
            if cap.isOpened():
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
                        #print(fps_samples)
                        i=0
                    screen.fill([0,0,0])
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    frame = np.rot90(frame)
                    frame = pygame.surfarray.make_surface(frame)
                    screen.blit(frame, (0,0))
                    #time.sleep(1 / FPS)
                pygame.display.update()
            #print("running")


        #screen.fill([0, 0, 0])
        #pygame.display.update()
        
        

    # Call it always before finishing. To deallocate resources.
    drone_swarm.drones_stop()
    if stream:
        cap.release()

if __name__ == '__main__':
    main()



