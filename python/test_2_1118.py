# -*- coding: utf-8 -*-
import cv2
import numpy as np
import serial
import sys
import time

font = cv2.FONT_HERSHEY_SIMPLEX

##--------------------- color_threshold --------------------

# red
r_low_hsv1  = np.array([156, 43,  130])
r_high_hsv1 = np.array([180, 255, 255])
r_low_hsv2  = np.array([0,   43,  130])
r_high_hsv2 = np.array([5,   255, 255])

# green
g_low_hsv1  = np.array([45,  33,  26 ])
g_high_hsv1 = np.array([80,  255, 255])

# blue
b_low_hsv1  = np.array([100, 43,  46 ])
b_high_hsv1 = np.array([124, 255, 255])

# white
w_low_hsv1  = np.array([0,   0,   200])
w_high_hsv1 = np.array([180, 43,  255])

##--------------------- camera_init --------------------

cap2 = cv2.VideoCapture(0)
cap2.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
cap1 = cv2.VideoCapture(2)
cap1.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))

##--------------------- PID && V_CONTROL --------------------

class PID():
    def __init__(self, max, min, Kp, Kd, Ki):
        self.max = max
        self.min = min
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.integral = 0
        self.pre_error = 0
 
    def calculate(self, setPoint, pv):
        error = setPoint - pv
        Pout = self.Kp * error
        self.integral += error
        Iout = self.Ki * self.integral
        derivative = error - self.pre_error
        Dout = self.Kd * derivative
        output = Pout + Iout + Dout
        if(output > self.max):
            output = self.max
        elif(output < self.min):
            output = self.min
        self.pre_error = error
        return output

class V_CONTROL:
    def __init__(self, max_to_1500, max_of_acc, feed):
        self.max_to_1500 = max_to_1500
        self.max_of_acc = max_of_acc
        self.feed = feed
        self.last_pwm = 1500
        
    def percent2str(self, vin):
        vin_feed_mul = vin * self.feed
        if vin_feed_mul > 100:
            vin_feed_mul = 100
        elif vin_feed_mul < -100:
            vin_feed_mul = -100
            
        now_pwm = int(1500 + (self.max_to_1500 * vin_feed_mul / 100))
        if now_pwm - self.last_pwm > self.max_of_acc:
            now_pwm = self.last_pwm + self.max_of_acc
        elif self.last_pwm - now_pwm > self.max_of_acc:
            now_pwm = self.last_pwm - self.max_of_acc
            
        self.last_pwm = now_pwm
        return str(now_pwm)
        
##--------------------- get_track --------------------

def get_track(cap, index):
    global r_low_hsv1, r_high_hsv1, r_low_hsv2, r_high_hsv2, road_cnt, time_cnt, state
    while True:
        try:
            success, img = cap.read()
            # 颜色转换函数 转换为hsv cv2.COLOR_BGR2HSV
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # mask是只突出指定颜色的图片
            mask1 = cv2.inRange(hsv, lowerb=r_low_hsv1, upperb=r_high_hsv1)
            mask2 = cv2.inRange(hsv, lowerb=r_low_hsv2, upperb=r_high_hsv2)
            if index == 2:
                mask = mask1 + mask2
            elif index == 1:
                mask = cv2.inRange(hsv, lowerb=g_low_hsv1, upperb=g_high_hsv1)
            # 中值滤波降噪
            median = cv2.medianBlur(mask, 5)
            """
            ---
            contours返回轮廓的点集
            ---
            hierachy返回N*4的矩阵， N表示轮廓个数
                    
            第一个数：表示同一级轮廓的下个轮廓的编号，如果这一级轮廓没有下一个轮廓，一般是这一级轮廓的最后一个的时候，则为-1
            第二个数：表示同一级轮廓的上个轮廓的编号，如果这一级轮廓没有上一个轮廓，一般是这一级轮廓的第一个的时候，则为-1
            第三个数：表示该轮廓包含的下一级轮廓的第一个的编号，假如没有，则为-1
            第四个数： 表示该轮廓的上一级轮廓的编号，假如没有上一级，则为-1
            """
            contours, hierarchy = cv2.findContours(median, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            #  cv2.RETR_EXTERNAL 只寻找最高级轮廓，即最外面的轮廓
            if len(contours) != 0:
                area = []
                # 找到最大的轮廓
                for k in range(len(contours)):
                    # contourArea面积计算
                    area.append(cv2.contourArea(contours[k]))

                # 面积最大轮廓的索引
                max_idx = np.argmax(np.array(area))

                # 生成最小的外界矩形
                rect = cv2.minAreaRect(contours[max_idx])

                # boxPoints返回四个点坐标
                box = cv2.boxPoints(rect)
                box = np.int0(box)  # 将坐标规范化为整数

                # 在opencv的坐标体系下，纵坐标最小的是top_point，纵坐标最大的是bottom_point， 横坐标最小的是left_point，横坐标最大的是right_point
                # 获取四个顶点坐标
                left_point_x = np.min(box[:, 0])
                right_point_x = np.max(box[:, 0])
                top_point_y = np.min(box[:, 1])
                bottom_point_y = np.max(box[:, 1])
                left_point_y = box[:, 1][np.where(box[:, 0] == left_point_x)][0]
                right_point_y = box[:, 1][np.where(box[:, 0] == right_point_x)][0]
                top_point_x = box[:, 0][np.where(box[:, 1] == top_point_y)][0]
                bottom_point_x = box[:, 0][np.where(box[:, 1] == bottom_point_y)][0]
                # 即获得矩形框四个点在opencv坐标体系下的各个点的值
                cv2.drawContours(img, [box], 0, (255, 0, 0), 3)
##_-------------------------------------------------------------------------------------------------
                cv2.line(img, (640-100, 0), (640-100, 480), (0, 255, 0), 5)
                cv2.line(img, (  0+100, 0), (  0+100, 480), (0, 255, 0), 5)
                
                if left_point_x > 640-100 or right_point_x < 0+100:
                    if road_cnt % 10 >= 5 and road_cnt % 10 <= 9:
                        road_cnt = road_cnt + 1
                    elif road_cnt % 10 >= 1 and road_cnt % 10 <= 4:
                        road_cnt = road_cnt - 1
                
                    return [0, 0, 0, 0, 0]

                ##-----                    
                elif bottom_point_y - top_point_y > 350:
                    if road_cnt % 10 >= 5 and road_cnt % 10 <= 9:
                        road_cnt = road_cnt + 1
                    elif road_cnt % 10 >= 1 and road_cnt % 10 <= 4:
                        road_cnt = road_cnt - 1
                    
                    if bottom_point_x - left_point_x == 0 or bottom_point_x - right_point_x == 0:
                        tmp_angle = 0
                    elif (bottom_point_x - left_point_x)  * (bottom_point_x - left_point_x)  + \
                         (bottom_point_y - left_point_y)  * (bottom_point_y - left_point_y)  < \
                         (bottom_point_x - right_point_x) * (bottom_point_x - right_point_x) + \
                         (bottom_point_y - right_point_y) * (bottom_point_y - right_point_y):
                        tmp_angle = int(rect[2])
                    else:
                        tmp_angle = int(rect[2] - 90)
                    tmp_x = 0
                    
                    cv2.circle(img, (320, 240 + tmp_x), 3, (0, 255, 0), 5)
                    cv2.line(img, (640, 240 + int(320 * np.tan(tmp_angle / 180 * np.pi))), \
                                  (  0, 240 - int(320 * np.tan(tmp_angle / 180 * np.pi))), \
                                    (0, 0, 255), 5)
                    cv2.putText(img, str(tmp_x) + ',' + str(tmp_angle), (400, 240), font, 1, (0, 0, 255), 2)                    
                    
                ##-----
                else:
                    if road_cnt % 10 >= 0 and road_cnt % 10 <= 4:
                        road_cnt = road_cnt + 1
                    elif road_cnt % 10 >= 6 and road_cnt % 10 <= 9:
                        road_cnt = road_cnt - 1
                        
                    if bottom_point_x - left_point_x == 0 or bottom_point_x - right_point_x == 0:
                        tmp_angle = 0
                    elif (bottom_point_x - left_point_x)  * (bottom_point_x - left_point_x)  + \
                         (bottom_point_y - left_point_y)  * (bottom_point_y - left_point_y)  < \
                         (bottom_point_x - right_point_x) * (bottom_point_x - right_point_x) + \
                         (bottom_point_y - right_point_y) * (bottom_point_y - right_point_y):
                        tmp_angle = int(rect[2] - 90)
                    else:
                        tmp_angle = int(rect[2])
                    tmp_x = -240 + int(np.tan(tmp_angle / 180 * np.pi) * (320 - rect[0][0]) + rect[0][1])
                    
                    cv2.circle(img, (320, 240 + tmp_x), 3, (0, 255, 0), 5)
                    cv2.line(img, (320, 240 + tmp_x), (320, 240), (0, 0, 255), 5)
                    cv2.putText(img, str(tmp_x) + ',' + str(tmp_angle), (400, 240), font, 1, (0, 0, 255), 2)
                
                bias_x = int(rect[0][0]) - 320
                bias_y = int(rect[0][1]) - 240
                d_x = right_point_x - left_point_x
                
                cv2.putText(img, str(       bias_x), (640-70,  60), font, 1, (0, 255, 0), 2)
                cv2.putText(img, str(       bias_y), (640-70, 120), font, 1, (0, 255, 0), 2)
                cv2.putText(img, str(          d_x), (640-70, 180), font, 1, (0, 255, 0), 2)
                
                cv2.putText(img, str(int(road_cnt)), (640-70, 240), font, 1, (0, 0, 255), 2)
                cv2.putText(img, str(     time_cnt), (640-70, 300), font, 1, (0, 0, 255), 2)
                cv2.putText(img, str(        state), (640-70, 360), font, 1, (0, 0, 255), 2)
 
                cv2.imshow('img', img)
                #cv2.imshow('median', median)
                
                return [tmp_x, tmp_angle, bias_x, bias_y, d_x]
                
            else:
                return [0, 0, 0, 0, 0]
        except:
            pass

##--------------------- PID && V_CONTROL --------------------

##----- V_CONTROL
vfeed_1 = 0.9
vfeed_2 = 1.1
vfeed_3 = 1
vfeed_4 = 1
vfeed_5 = 1
vfeed_6 = 1
v_z     = 0
v_pitch = 0
v_x     = 0
v_y     = 0
v_p     = 0
max_to_1500 = 200
max_of_acc  = 50
v1 = V_CONTROL(max_to_1500, max_of_acc, vfeed_1)
v2 = V_CONTROL(max_to_1500, max_of_acc, vfeed_2)
v3 = V_CONTROL(max_to_1500, max_of_acc, vfeed_3)
v4 = V_CONTROL(max_to_1500, max_of_acc, vfeed_4)
v5 = V_CONTROL(max_to_1500, max_of_acc, vfeed_5)
v6 = V_CONTROL(max_to_1500, max_of_acc, vfeed_6)

##----- PID
x_pid     = PID(60, -60, 0.3, 0, 0)
y_pid     = PID(60, -60, 0.3, 0, 0)
p_pid     = PID(20, -20, 0.6, 0, 0)
z_pid     = PID(50, -50, 10,  100, 0)
pitch_pid = PID(0,  -0,  0.1, 2, 0)

pid_state = 0
##----- 0 : y_pid on;  p_pid on;  v_x adjust from v_y, v_p.
##----- 1 : y_pid_off; p_pid_off; x_pid_off.
##----- 2 : y_pid_on;  p_pid_on;  x_pid_on.

##--------------------- Varies_from_sensors --------------------

del_x = 0
rho   = 0
angle = 0
depth = 0
pitch = 0

##--------------------- Varies_of_state --------------------

road_cnt = 0
time_cnt = 0

state = 0
##----- 0 : track_line_1
##----- 1 : track_ball
##------2 : +x
##----- 3 : -x && -y
##----- 4 : track_line_2
##----- 5 : get_ball

##----------------------------------------------------

text = sys.argv[1]
ser = serial.Serial("/dev/ttyAMA1", 115200, timeout = 0.5)
ser.write((text+'\r\n').encode()) 

##----------------------------------------------------

a_depth = 18 + 10
if cap1.isOpened() and cap2.isOpened():
    while True:
        if  (state == 0):
            track_res = get_track(cap2, 2)
            rho   = track_res[0]
            angle = track_res[1]
            del_x = track_res[2]
            
            if  (road_cnt >= 0 and road_cnt <= 10 and del_x < -100):
                a_depth = 18 + 10
            elif(road_cnt > 10 and road_cnt <= 20 and del_x < -100):
                a_depth = 18
            elif(road_cnt > 20 and road_cnt <= 30 and del_x < -100):
                a_depth = 18 + 10
            elif(road_cnt > 30 and road_cnt <= 40 and del_x < -100):
                a_depth = 18
            
            if road_cnt >= 41:
                road_cnt = 0
                state = 1
        
        ##-----
        elif(state == 1):
            track_res = get_track(cap1, 1)
            rho   = 0
            angle = track_res[2] / 20
            del_x = track_res[4]
            
            if del_x > 500:
               state = 2
        
        ##-----
        elif(state == 2):
            track_res = get_track(cap1, 1)
            rho   = 0
            angle = track_res[2] / 30
            del_x = track_res[4]
               
            time_cnt = time_cnt + 1
            if time_cnt > 10:
                time_cnt = 0
                state = 3
         
        ##-----     
        elif(state == 3):
            track_res = get_track(cap2, 2)
            v_x = -25
            v_y = 0
            v_p = 0
            pid_state = 1
            
            time_cnt = time_cnt + 1
            if time_cnt > 60:
                contain = 0
                time_cnt = 0
                state = 4
                
        ##-----     
        elif(state == 4):
            track_res = get_track(cap2, 2)
            v_x = 0
            v_y = 0
            v_p = -25
            pid_state = 1
            
            time_cnt = time_cnt + 1
            if time_cnt > 10:
                pid_state = 0
                road_cnt = 0
                time_cnt = 0
                state = 5
                 
        ##-----
        elif(state == 5):
            track_res = get_track(cap2, 2)
            rho   = track_res[0]
            angle = track_res[1]
            del_x = track_res[2]
            
            if road_cnt >= 15:
                road_cnt = 0
                pid_state = 0
                state = 6
        
        ##-----
        elif(state == 6):
            track_res = get_track(cap1, 1)
            rho   = 0
            angle = track_res[2] / 30
            depth = a_depth + track_res[3]
            del_x = track_res[4] / 30
            
            if depth < 20:
                depth = 20
            
#----------------------------------------------------------------------------------    

        size = ser.inWaiting()
        if size != 0:
            response = ser.read_all().split()
            if len(response) >= 6:
                if response[2][:-1].isdigit():
                    depth = int(response[2][:-1])
                if response[5].isdigit() or response[5][1:].isdigit():
                    pitch = int(response[5])
            ser.flushInput()
                  
#----------------------------------------------------------------------------------
             
        v_z     =     z_pid.calculate(a_depth, depth)
        v_pitch = pitch_pid.calculate(0, pitch)
        
        if   pid_state == 0:
            v_y     =    -y_pid.calculate(0, rho)
            v_p     =    -p_pid.calculate(0, angle)        
            v_x     = 25 - 0.5 * (abs(v_y) + abs(v_p))
            if v_x < 10:
                v_x = 10
        elif pid_state == 2:
            v_y     =    -y_pid.calculate(0, rho)
            v_p     =    -p_pid.calculate(0, angle)
            v_x     =     x_pid.calculate(0, del_x)            
           
        v_p = v_p - 4
        v_y = v_y + 2      
        str1 = v1.percent2str(v_z + v_pitch)
        str2 = v2.percent2str(v_z - v_pitch)
        str3 = v3.percent2str(v_x + v_y + v_p)
        str4 = v4.percent2str(v_x - v_y - v_p)
        str5 = v5.percent2str(v_x + v_y - v_p)
        str6 = v6.percent2str(v_x - v_y + v_p)
        
#----------------------------------------------------------------------------------       
 
        ser.write(('0,0,' + str1 + ',' + str2 + ',' + str3 + ',' + str4 + ',' + str5 + ',' + str6).encode())
        
        print(        \
                      str(           depth).ljust(3,' ') + \
       #              str(           pitch).ljust(4,' ') + \
        ' | x '     + str(        int(v_x)).ljust(3,' ') + \
        ' | z '     + str(        int(v_z)).ljust(3,' ') + \
       #' | pitch ' + str(    int(v_pitch)).ljust(3,' ') + \
        ' | y '     + str(        int(v_y)).ljust(3,' ') + \
        ' | p '     + str(        int(v_p)).ljust(3,' ') + \
        ' | pwm '   + str(int(str1) - 1500).ljust(5,' ') + \
                      str(int(str2) - 1500).ljust(5,' ') + \
                      str(int(str3) - 1500).ljust(5,' ') + \
                      str(int(str4) - 1500).ljust(5,' ') + \
                      str(int(str5) - 1500).ljust(5,' ') + \
                      str(int(str6) - 1500).ljust(5,' ') + \
                      ' ' )
        
        if cv2.waitKey(1) & 0xff == ord('q'):
            break
else:
    print("error open cap2 failed")


time.sleep(0.1)   
ser.write(('0,0,0,0,0,0,0,0,0').encode())
