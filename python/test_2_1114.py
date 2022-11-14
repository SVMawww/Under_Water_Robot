# -*- coding: utf-8 -*-
import cv2
import numpy as np
import serial
import sys
import time

font = cv2.FONT_HERSHEY_SIMPLEX

##-----------------------------------------------------------
# red
r_low_hsv1  = np.array([156, 43,  130])
r_high_hsv1 = np.array([180, 255, 255])
r_low_hsv2  = np.array([0,   43,  130])
r_high_hsv2 = np.array([5,   255, 255])

# green
g_low_hsv1  = np.array([35,  43,  46 ])
g_high_hsv1 = np.array([77,  255, 255])

# blue
b_low_hsv1  = np.array([100, 43,  46 ])
b_high_hsv1 = np.array([124, 255, 255])

# white
w_low_hsv1  = np.array([0,   0,   200])
w_high_hsv1 = np.array([180, 43,  255])

##-----------------------------------------------------------

cap2 = cv2.VideoCapture(0)
cap2.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
cap1 = cv2.VideoCapture(2)
cap1.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))

road_cnt = 0

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
    global r_low_hsv1, r_high_hsv1, r_low_hsv2, r_high_hsv2, road_cnt
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
                """
                rect[0]返回矩形的中心点，（x,y），实际上为y行x列的像素点
                
                rect[1]返回矩形的长和宽，顺序一定不要弄错了
                
                rect[2]返回矩形的旋转角度
                
                旋转角度θ是水平轴（x轴）逆时针旋转，直到碰到矩形的第一条边停住，此时该边与水平轴的夹角。并且这个边的边长是width，另一条边边长是height。也就是说，在这里，width与height不是按照长短来定义的。
                angel是由x轴逆时针转至W(宽)的角度。
                 角度范围是[-90,0) 
                """
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
##_----------------------------------------------------------            
                cv2.line(img, (640-150, 0), (640-150, 480), (0, 255, 0), 5)
                cv2.line(img, (  0+150, 0), (  0+150, 480), (0, 255, 0), 5)
                
                if left_point_x > 640-150 or right_point_x < 0+150:
                    return None

                ##-----                    
                elif bottom_point_y - top_point_y > 400:
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
                        
                cv2.putText(img, str(road_cnt), (640-70, 240), font, 1, (0, 255, 0), 2)
##_-----------------------------------------------------------
                cv2.imshow('img', img)
                cv2.imshow('median', median)
                
                return [tmp_x, tmp_angle, int(rect[0][0])]
            else:
                #cv2.imshow('img', img)
                #cv2.imshow('median', median)
                return None
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
v_x     = 18
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
y_pid     = PID(50, -50, 1,   0, 0)
p_pid     = PID(30, -30, 0.5, 0, 0)
z_pid     = PID(30, -30, 30,  0, 0)
pitch_pid = PID(30, -30, 0.2, 2, 0)

##--------------------- Varies_from_sensors --------------------

rho   = 0
angle = 0
depth = 0
pitch = 0

##----------------------------------------------------

text = sys.argv[1]
ser = serial.Serial("/dev/ttyAMA1", 115200, timeout=0.5)
ser.write((text+'\r\n').encode())

##----------------------------------------------------

if cap1.isOpened() and cap2.isOpened():
    while True:
        if  (road_cnt >= 0 and road_cnt <= 10):
            a_depth = 40
        elif(road_cnt > 10 and road_cnt <= 20):
            a_depth = 25
        elif(road_cnt > 20 and road_cnt <= 30):
            a_depth = 40
        elif(road_cnt > 30 and road_cnt <= 40):
            a_depth = 25
        a_depth = 40
        
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
        
        if road_cnt < 50:
            track_res = get_track(cap2, 2)
        else:
            track_res = get_track(cap1, 1)
        
        if track_res != None:
            rho = track_res[0]
            angle = track_res[1]
        else:
            rho = 0
            angle = 0
                  
#----------------------------------------------------------------------------------
             
        v_z     =     z_pid.calculate(a_depth, depth)
        v_pitch = pitch_pid.calculate(0, pitch)
        v_y     =    -y_pid.calculate(0, rho)
        v_p     =    -p_pid.calculate(0, angle)

        str1 = v1.percent2str(v_z + v_pitch)
        str2 = v2.percent2str(v_z - v_pitch)
        str3 = v3.percent2str(v_x + v_y + v_p)
        str4 = v4.percent2str(v_x - v_y - v_p)
        str5 = v5.percent2str(v_x + v_y - v_p)
        str6 = v6.percent2str(v_x - v_y + v_p)
        
#----------------------------------------------------------------------------------       
 
        ser.write(('0,0,' + str1 + ',' + str2 + ',' + str3 + ',' + str4 + ',' + str5 + ',' + str6).encode())
        
        print(        \
                      str(           depth).ljust(2,' ') + \
                      str(           pitch).ljust(4,' ') + \
        ' | z '     + str(        int(v_z)).ljust(3,' ') + \
        ' | pitch ' + str(    int(v_pitch)).ljust(3,' ') + \
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
