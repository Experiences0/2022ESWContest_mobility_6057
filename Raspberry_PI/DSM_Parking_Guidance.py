#!/usr/bin/env python3
import serial

import threading
import time
import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2 as cv
import math
import pygame
import queue

NO_OF_MESSAGE=10
msgQ=queue.Queue(NO_OF_MESSAGE)
msgQ_Radius=queue.Queue(NO_OF_MESSAGE)


class Arduino:
    def __init__(self):
        self.comm = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        self.isWoring = True
        self.angle=0
        self.distance=0
        
        self.comm.flush()
        self. ultrasonic_xy=[0.315,0.19]
        
        
    def read(self):
        self.comm.reset_input_buffer()
        try:
            output = self.comm.readline()
            output = str(output)
        
            output=output.replace("b'","")
            output=output.replace("\\r\\n'","")
            data=output.split(',')
            self.distance=float(data[0])/1000
            self.angle=float(data[1])
            
        except:
            self.distance=10000
            
        
       
    def calc_minimum(self,obs_depth):
        obs_ultra=[[self.ultrasonic_xy[0],self.ultrasonic_xy[1]+self.distance]]
        
        if obs_depth[0][1] > obs_ultra[0][1]:
           
            return obs_ultra
        else:
            return obs_depth
    
    
   
    
    def Alarm_arduino(self,alarm_level):
    
            if alarm_level==3:
                self.comm.write(str.encode('c'))
          
            elif alarm_level==2:
                self.comm.write(str.encode('b'))
              
            elif alarm_level==1:
                self.comm.write(str.encode('a'))
            
            else:
                self.comm.write(str.encode('d'))
                

class Drive_Rad_Calculator:
    def __init__(self,L,P,W,B,Door_L,Door_max_angle):
        self.L = L;
        self.P = P;
        self.W = W;
        self.B = B;
        self.Door_L=Door_L;
        self.Door_xy=[0.39,0.23]
        self.Door_max_angle=Door_max_angle
   
        self.Steering_Angle=0;
        self.R1=0;
        self.R1_=0;
        self.R2=0;
        self.R2_=0;
        self.R_abs=[0,0,0,0];
        self.Centroid=0;
        self.margin=0;
        self.Gudian_R=0;
        self.crashable=[0,0];
  
    def Calc_Driving_Radius(self,angle):
        self.Steering_Angle = angle*3.141592/180.0

        if (abs(self.Steering_Angle)) < 0.000001:
            
            self.R_abs=[100,100,100+self.L,100+self.L];

        else:
            
            gain = 0.1
            self.R1 = self.W * 1 / math.sin(self.Steering_Angle) + (self.L - self.P) / 2 + ((self.B-self.L)/2) + gain
            self.R1_ = self.W * 1 / math.tan(self.Steering_Angle) + (self.L - self.P) / 2 + ((self.B-self.L)/2) + gain 
            self.R2 = self.W * 1 / math.sin(self.Steering_Angle) - (self.L - self.P) / 2 - ((self.B-self.L)/2) - gain
            self.R2_ = self.W * 1 / math.tan(self.Steering_Angle) - (self.L - self.P) / 2 - ((self.B-self.L)/2) - gain
            
            self.R_abs=[abs(self.R1),abs(self.R1_),abs(self.R2_),abs(self.R2)]
            
            self.Centroid=(self.R1_+self.R2_)/2
            
        self.R_min=min(self.R_abs)
        self.R_max = max(self.R_abs)


    def Calc_Crashable_and_Guidance(self,obstacles):
        x=obstacles[0][1]
        y=obstacles[0][0]
        
        if (self.R_min-self.margin)**2 < ((x-(self.R_min+self.R_max)/2)**2+(y)**2) and (self.R_max+self.margin)**2 > ((x-(self.R_min+self.R_max)/2)**2 + (y**2)) :
            self.crashable[0] = 1
            self.Gudian_R= ( x**2 + y**2 + (self.L/2)**2 - x*self.L ) / ( 2*x - self.L) + (self.B-self.L)/2 + 0.4
        else:
            self.crashable[0] = 0


        self.crashable[1] = obstacles[0][1]

 
    def Calc_Door_Ding(self,obstacles):
        a=[-obstacles[0][0],0]
        b=[obstacles[0][0]-self.Door_xy[0],obstacles[0][1]-self.Door_xy[1]]
        
        tmp=(math.sqrt(a[0]**2+a[1]**2)*math.sqrt(b[0]**2+b[1]**2))
        if tmp<0.0001:
            pass
        else:
            
            ob_angle=(180/3.141592)*math.acos((a[0]*b[0]+a[1]*b[1])/tmp)+5
            
            if ob_angle < self.Door_max_angle:
                return ob_angle
            else:
                return -1
            
    def Calc_Cornering_Alarm(self):
        ratio = self.Gudian_R/self.R_min
        
        if self.crashable[0]==1:
            if ratio<0.6:
                alarm_level=1
            elif ratio<0.8:
                alarm_level=2
            else:
                alarm_level=3
        else:
            alarm_level=0
            
        return alarm_level
            
                
    def Calc_Sidemirror_Ding(self,ob):

            
        distance = 0.54-ob[0][0]
        if ob[0][1]<0.35:
            if distance < 0.15 :
                sidemirror_alarm_level = 3
            elif distance <0.25:
                sidemirror_alarm_level = 2
      
            else :
                sidemirror_alarm_level = 0
        else:
            sidemirror_alarm_level = 0
        
        return sidemirror_alarm_level
    
     



# ------------------------------depth camera 함수 설정--------------------------------- #

# 카메라 초기 설정
def camera_init():

    global clipping_distance

    pl = rs.pipeline()
    config = rs.config()
    pipeline_wrapper = rs.pipeline_wrapper(pl)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    # Depth 해상도, RGB 해상도 결정
    config.enable_stream(rs.stream.depth, d_width, d_height, rs.format.z16, 60)
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 15)

    # 카메라 start
    profile = pl.start(config)

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    # print("Depth Scale is: ", depth_scale)

    clipping_distance_in_meters = 1  # 1meter
    clipping_distance = clipping_distance_in_meters / depth_scale

    align_to = rs.stream.color
    ali = rs.align(align_to)

    return pl, ali


# Pixel to mm 좌표 획득
def find_coordinate(min_x, min_y, depth):
    global width, height

    x = depth * math.tan(26.8596 * math.pi / 180) * (min_x - width / 2) / (width / 2)
    y = depth * math.tan(20.6854 * math.pi / 180) * (height / 2 - min_y) / (height / 2)
    z = depth * (-1)

    return x, y, z


# Camera -> Car 좌표계로의 좌표 변환
def convert_coordinate(points, matrix):

    final_points = []

    if not points:
        return

    prev_points = np.hstack((np.array(points), np.ones((len(points), 1))))

    # 1차원 행렬의 경우 transpose 를 해주지 않아도 된다.
    for i in range(len(points)):
        final_points.append(matrix@np.array(prev_points[i]))

    # 끝에 붙어있는 1을 다시 삭제하여 (x, y, z) 좌표로 반환
    final_points = np.delete(final_points, 3, 1)

    return final_points


# 장애물 edge(사이드미러와 가장 가까운 점) (x,y,z) 획득
def detecting_edge(screen, line_index, draw_image):

    global clipping_distance

    min_y, min_z = 0, 1000
    xyz = []

    for n in range(len(line_index)):

        for i in range(len(screen)):
            depth = screen[i][line_index[n]]
            if (depth > 0) and (depth <= clipping_distance):
                if depth < min_z:
                    min_z = depth
                    min_y = i
            else:
                pass

        if (min_z > 0) and (min_z <= clipping_distance):
            cv.circle(draw_image, (line_index[n], min_y), radius=5, color=(0, 0, 255), thickness=-1)
            cv.putText(draw_image, "{}mm".format(min_z), (line_index[n], min_y - 20), cv.FONT_HERSHEY_PLAIN, 2,
                       (255, 0, 0), 2)

        ed_x, ed_y, ed_z = find_coordinate(line_index[n], min_y, min_z)
        xyz.append([ed_x, ed_y, ed_z])

    new_xyz = convert_coordinate(xyz, Homo_trans)

    return new_xyz


# 자동차와 가장 가까운 점 (x,y,z) 획득
def detecting_nearest(screen, line_index, draw_image):

    global ground_offset

    near_x, near_y, near_z, draw_y = 0, 1000, 0, 0
    near_xyz = []

    for n in range(len(line_index)):

        for i in range(50, len(screen)):
            depth = screen[i][line_index[n]]

            x, y, z = find_coordinate(line_index[n], i, depth)
            world_xyz = convert_coordinate([[x, y, z]], Homo_trans)
            if (depth > 0) and (depth <= clipping_distance) and world_xyz[0][1] > 0 and world_xyz[0][2] < 360 - ground_offset:
                if world_xyz[0][1] <= near_y:
                    near_x = world_xyz[0][0]
                    near_y = world_xyz[0][1]
                    near_z = world_xyz[0][2]
                    draw_y = i
        if draw_y:
            cv.circle(draw_image, (line_index[n], draw_y), radius=5, color=(255, 0, 0), thickness=-1)
            cv.putText(draw_image, "y:{}mm".format(int(near_y)), (line_index[n] - 50, draw_y - 20), cv.FONT_HERSHEY_PLAIN, 2,
                       (255, 0, 0), 2)

        near_xyz.append([near_x/1000, near_y/1000, near_z/1000])

    return near_xyz


# depth_image 와 color_image 획득
def get_frame(pip, al):

    global clipping_distance

    frames = pip.wait_for_frames()
    aligned_frames = al.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    #
    # if not aligned_depth_frame or not color_frame:
    #     continue

    dep_image = np.asanyarray(aligned_depth_frame.get_data())
    col_image = np.asanyarray(color_frame.get_data())

    grey_color = 153
    depth_image_3d = np.dstack((dep_image, dep_image, dep_image))
    bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, col_image)
    bg_removed = cv.rotate(bg_removed, cv.ROTATE_90_CLOCKWISE)

    cv.imshow('Depth Camera', bg_removed)

    return dep_image, col_image


# 보기에 용이하도록 시계 방향으로 90도 회전
def view_rgb(img):
    img = cv.rotate(img, cv.ROTATE_90_CLOCKWISE)
    cv.imshow('RGB Camera', img)


# ------------------------------변수 설정--------------------------------- #


# 해상도 결정(d_** : depth 해상도, **: RGB 해상도)
d_width, d_height = 480, 270
width, height = 640, 480

# 좌표변환 행렬(Camera->Car)
Homo_trans = np.array([[0.1651, -0.3090, 0.9366, 545], [-0.0537, -0.9511, -0.3043, 240], [0.9848, 0, -0.1736, 0], [0, 0, 0, 1]])

# Maximum Depth 제한, camera_init 후 값 갱신
clipping_distance = 0

# 카메라로 바닥을 식별하지 않기 위한 offset 설정 (바닥으로부터의 거리)
ground_offset = 30

# 카메라 초기 설정
pipeline, align = camera_init()

# Edge 와 가장 가까운 곳의 x 좌표 list 설정
#edge_x_list = [200, 300, 400]
edge_x_list = [400]
#nearest_x_list = [450, 550]
nearest_x_list = [450]

        

##========================== GUI ==============================


# global var
s_ticks = None
s_toggle = True

# screen size
screen_width = 800
screen_height = 415

pygame.init()
screen = pygame.display.set_mode((screen_width,screen_height))
font = pygame.font.SysFont("arial",30,True,True)
sfont = pygame.font.SysFont("arial",20)

# door image load
carimg = pygame.image.load("car_img.png")
carimg = pygame.transform.scale(carimg,(800,415))


def convert_degree(r):
    # car wheel num
    # == front ==
    #  0 <140> 1   ^146
    #  2       3
    # ==  back  ==
    num = -1
    distance = 0
    length = [70,100]
    #print('r',r)
    if r>1970 or r<-1970:
        pygame.draw.line(screen, (0,0,255),[107+5,75+66],[107+5,75+66-80],7)
        pygame.draw.line(screen, (0,0,255),[107+140-5,75+66],[107+140-5,75+66-80],7)
    elif r>0:
        # left front
        distance = int(math.sqrt((r+70)**2+146**2))
        pygame.draw.arc(screen,(0,0,255),[107+70+r-distance,75+212-distance,distance*2,distance*2],math.pi-math.asin((75+66)/distance)-math.asin(length[1]/distance),math.pi-math.asin((75+66)/distance),5)
        # right front
        distance = int(math.sqrt((r-70)**2+146**2))
        pygame.draw.arc(screen,(0,0,255),[107+70+r-distance-6,75+212-distance,distance*2,distance*2],math.pi-math.asin(130/distance)-math.asin(length[1]/distance),math.pi-math.asin(130/distance),5)
        # right rear
        distance = r-70
        pygame.draw.arc(screen,(0,0,255),[107+140-6,75+212-distance,distance*2,distance*2],math.pi-math.asin(length[1]/distance),math.pi,5)
    elif r<0:
        r=-r
        # left front
        distance = int(math.sqrt((r-70)**2+146**2))
        pygame.draw.arc(screen,(0,0,255),[107+70-r-distance+6,75+212-distance,distance*2,distance*2],math.asin((75+66)/distance),math.asin((75+66)/distance)+math.asin(length[1]/distance),5)
        # right front
        distance = int(math.sqrt((r+70)**2+146**2))
        pygame.draw.arc(screen,(0,0,255),[107+70-r-distance,75+212-distance,distance*2,distance*2],math.asin((75+66)/distance),math.asin((75+66)/distance)+math.asin(length[1]/distance),5)
        # left rear
        distance = r-70
        pygame.draw.arc(screen,(0,0,255),[107-2*distance+6,75+212-distance,distance*2,distance*2],0,math.asin(length[1]/distance),5)

def guide_degree(r):
    # car wheel num
    # == front ==
    #  0 <140> 1   ^146
    #  2       3
    # ==  back  ==
    num = -1
    distance = 0
    length = [70,100]
    #print('r',r)
    if r>1970 or r<-1970:
        pygame.draw.line(screen, (0,255,0),[107+5,75+66],[107+5,75+66-80],7)
        pygame.draw.line(screen, (0,255,0),[107+140-5,75+66],[107+140-5,75+66-80],7)
    elif r>0:
        # left front
        distance = int(math.sqrt((r+70)**2+146**2))
        pygame.draw.arc(screen,(0,255,0),[107+70+r-distance,75+212-distance,distance*2,distance*2],math.pi-math.asin((75+66)/distance)-math.asin(length[1]/distance),math.pi-math.asin((75+66)/distance),5)
        # right front
        distance = int(math.sqrt((r-70)**2+146**2))
        pygame.draw.arc(screen,(0,255,0),[107+70+r-distance-6,75+212-distance,distance*2,distance*2],math.pi-math.asin(130/distance)-math.asin(length[1]/distance),math.pi-math.asin(130/distance),5)
        # right rear
        distance = r-70
        pygame.draw.arc(screen,(0,255,0),[107+140-6,75+212-distance,distance*2,distance*2],math.pi-math.asin(length[1]/distance),math.pi,5)
    elif r<0:
        r=-r
        # left front
        distance = int(math.sqrt((r-70)**2+146**2))
        pygame.draw.arc(screen,(0,255,0),[107+70-r-distance+6,75+212-distance,distance*2,distance*2],int(math.asin((75+66)/distance)),int(math.asin((75+66)/distance)+math.asin(length[1]/distance)),5)
        # right front
        distance = int(math.sqrt((r+70)**2+146**2))
        pygame.draw.arc(screen,(0,255,0),[107+70-r-distance,75+212-distance,distance*2,distance*2],int(math.asin((75+66)/distance)),int(math.asin((75+66)/distance)+math.asin(length[1]/distance)),5)
        # left rear
        distance = r-70
        pygame.draw.arc(screen,(0,255,0),[107-2*distance+6,75+212-distance,distance*2,distance*2],0,math.asin(length[1]/distance),5)


def draw_edge_point(x,y):
    x,y=y+107+75,75+212-x
    pygame.draw.rect(screen, (255,0,0),[x,y,20,5],0)
    pygame.draw.rect(screen, (255,0,0),[x,y,5,20],0)
    
def warning(n):

    global s_ticks, s_toggle

    if s_ticks == None:

        s_ticks = pygame.time.get_ticks()

    
    if n == -1:

        elapsed_time = (pygame.time.get_ticks()-s_ticks)/1000

        if s_toggle:

            pygame.draw.circle(screen,(255,0,0),[107+145,75+85],20)

            if elapsed_time > 0.5:

                s_ticks = pygame.time.get_ticks()

                s_toggle = False

        elif not s_toggle:

            if elapsed_time > 0.5:

                s_ticks = pygame.time.get_ticks()

                s_toggle = True



    elif n == 0:

        surface = screen.convert_alpha()

        surface.fill((0,0,0,0))

        screen.blit(surface,(0,0))

        

    else:

        elapsed_time = (pygame.time.get_ticks()-s_ticks)/1000

        
        if s_toggle:

            surface = screen.convert_alpha()

            surface.fill((255,0,0,80))

            screen.blit(surface,(0,0))

            if elapsed_time > n:

                s_ticks = pygame.time.get_ticks()

                s_toggle = False

        elif not s_toggle:

            surface = screen.convert_alpha()

            surface.fill((0,0,0,0))

            screen.blit(surface,(0,0))

            msg = font.render("!WARNING!",True,(255,0,0))

            screen.blit(msg, (310,50))

            if elapsed_time > n:

                s_ticks = pygame.time.get_ticks()

                s_toggle = True

            

def status_display(encoder, radian, edge, door_degree, warning):

    stat = sfont.render("< STATUS >",True,(0,0,0))
    en = sfont.render("steering : "+str(encoder),True,(0,0,0))
    rad = sfont.render(f'R : {round(radian,4)}',True,(0,0,0))
    #ed = sfont.render(f'edge : {edge}',True,(0,0,0))
    dg = sfont.render(f'door : {door_degree}',True,(0,0,0))


    screen.blit(stat,(310,125))
    screen.blit(en, (310,160))
    screen.blit(rad, (310,200))
    #screen.blit(ed, (310,240))
    screen.blit(dg, (310,240))
            

def gui_running(r,x,y,guide_r,level,door_angle):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    screen.blit(carimg,(0,0))

    draw_edge_point(x, y) # x,y
    convert_degree(int(r))
    
    
    d = (door_angle)/2 
    #pygame.draw.line(screen,(120,120,10),[107+135,75+80],[107+135+120*math.sin(d*math.pi/180),75+80+120*math.cos(d*math.pi/180)],7)
    
    if level == 0 :
        pass
    if level >= 1 :
        guide_degree(int(guide_r))
        warning(1)
    elif level ==-1:
        warning(-1)
    

    # now pic car bound box (140,265)  zoom (2.55, 2.65)
    pygame.draw.rect(screen,(255,0,0),(107,75,139,266),1)
    # depth cam bound box (480,640) > (311.25,415)
    pygame.draw.rect(screen,(0,0,0),(screen_width-312,0,screen_width,screen_height),1)

    # ====== WARNING FLOAT to INT ======
    # front wheel location
    pygame.draw.rect(screen,(0,255,0),(107,75+66,139,1))
    # rear wheel location
    pygame.draw.rect(screen,(0,255,0),(107,75+212,139,1))
    
    
    status_display(arduino.angle,car_Driving.R_min,3,int(door_angle),5) # angle, radius, edge ,ding angle, level


    pygame.display.update()
    

#==============  TASK =======================

def TASK_DEPTH():
    while True:
        depth_image, color_image = get_frame(pipeline, align)
        edge_xyz = detecting_edge(np.asarray(depth_image), edge_x_list, color_image)
        nearest_xyz = detecting_nearest(np.asarray(depth_image), nearest_x_list, color_image)

        msgQ.put(nearest_xyz)
        
        view_rgb(color_image)
        time.sleep(0.1)
        
        
def TASK_GUI():
    while True:
        try:
            gui_running(car_Driving.Centroid*100*2.6,10,10)
            
        except:
            pass

        time.sleep(0.25)
level=0  
if __name__ == '__main__':


    arduino=Arduino()
    car_Driving=Drive_Rad_Calculator(0.425,0.295,0.530,0.51,0.3,74)
    
    def DRIVING_TASK():
        global level
        while True:
        
            arduino.read()
            time.sleep(0.05)

            car_Driving.Calc_Driving_Radius(arduino.angle)
            
            nearst_xyz=msgQ.get()
            
            _nearst_xyz=arduino.calc_minimum(nearst_xyz);
        

            car_Driving.Calc_Crashable_and_Guidance(_nearst_xyz)
        
            
            side_alarm=car_Driving.Calc_Sidemirror_Ding(_nearst_xyz)

            
            #=============  SIDE MIRROR ALARM PART ===============
            
            '''
            arduino.Alarm_arduino(side_alarm)
            if side_alarm>0 :
                level=-1
            else:
                level=0
            '''
            
             #=============  CORNERING ALARM PART ===============
            level=car_Driving.Calc_Cornering_Alarm()
            
            arduino.Alarm_arduino(level)

           
            

    Drive_Task=threading.Thread(target=DRIVING_TASK)
    Drive_Task.start()
    
    Depth_reader=threading.Thread(target=TASK_DEPTH)
    Depth_reader.start()
    
    #Gui_Task=threading.Thread(target=TASK_GUI)
    #Gui_Task.start()

    
    while True:
 
        time.sleep(0.25)
        nearst_xyz=msgQ.get()
        try:
            door_angle=car_Driving.Calc_Door_Ding(nearst_xyz)
            k=100*2.6
            guide_r=(car_Driving.Gudian_R + car_Driving.L/2 + 1.5 )*k 
        
            gui_running(car_Driving.Centroid*k,nearst_xyz[0][0]*k,nearst_xyz[0][1]*k,guide_r,level,door_angle)
        except:
            pass

        

        key = cv.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv.destroyAllWindows()
            pipeline.stop()
            break




