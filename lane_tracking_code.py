# -*- coding: utf-8 -*-
"""elec424projectWALLEE.py
Team WALL EE: Elijah Schwartz, Fredy Martinez, Nayana Chandrashekara Sharma, Juan Cano Duenas

citation:
code modifed from User raja_961, Autonomous Lane-Keeping Car Using Raspberry Pi and OpenCV. Instructables. URL: https://www.instructables.com/Autonomous-Lane-Keeping-Car-Using-Raspberry-Pi-and/ 
"""

import cv2
import numpy as np
import math
import sys
import time
import Adafruit_BBIO.PWM as PWM

#Initialize throttle
throttlePin = "P9_16" #check reference pic to locate
PWM.start(throttlePin, 7.5, 50)
#Initialize steering
steeringPin = "P9_14" #check reference pic to locate
PWM.start(steeringPin, 7.5, 50)

#Steering info
#duty cycle < 7.5 => right
#duty cycle = 7.5 => straight
#duty cycle > 7.5 => left 
#PWM.set_duty_cycle(steeringPin, duty_cycle)

#Throttle info
#duty cycle = 7.5 => stop
#duty cycle > 7.5 => forward
#do not exceed duty cycle of 8
#PWM.set_duty_cycle(throttlePin, duty_cycle)


def detect_edges(frame):
    # filter for blue lane lines
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #cv2.imshow("HSV",hsv)
    lower_blue = np.array([90, 120, 0], dtype = "uint8")
    upper_blue = np.array([150, 255, 255], dtype="uint8")
    mask = cv2.inRange(hsv,lower_blue,upper_blue)
    #cv2.imshow("mask",mask)
    
    # detect edges
    edges = cv2.Canny(mask, 50, 100)
    #cv2.imshow("edges",edges)
    
    return edges

def region_of_interest(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)

    # only focus lower half of the screen
    polygon = np.array([[
        (0, height),
        (0,  height/2),
        (width , height/2),
        (width , height),
    ]], np.int32)
    
    cv2.fillPoly(mask, polygon, 255)
    
    cropped_edges = cv2.bitwise_and(edges, mask)
    #cv2.imshow("roi",cropped_edges)
    
    return cropped_edges

def detect_line_segments(cropped_edges):
    rho = 1  
    theta = np.pi / 180  
    min_threshold = 10  
    
    line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold, 
                                    np.array([]), minLineLength=5, maxLineGap=150)

    return line_segments

def detect_stopsign(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    
    #red color has two ranges in hsv
    lower_red1 = np.array([0, 70, 20])
    upper_red1 = np.array([15, 255, 255])
    
    lower_red2 = np.array([150, 70,20])
    upper_red2 = np.array([179,255,255])
    
    red_mask_lower = cv2.inRange(hsv, lower_red1, upper_red1)
    red_mask_upper = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = red_mask_lower + red_mask_upper
    red = cv2.bitwise_and(image, image, mask = red_mask)
    total_pixels = red.size
    #if there is a significant amount of red, recognize it as a stopsign/light
    red_pixels = np.count_nonzero(red)
    percent = (red_pixels / total_pixels) * 100
    if percent > 25:
        return True
    return False


def average_slope_intercept(frame, line_segments):
    lane_lines = []
    
    if line_segments is None:
        print("no line segments detected")
        return lane_lines

    height, width,_ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1/3
    left_region_boundary = width * (1 - boundary)
    right_region_boundary = width * boundary
    
    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                print("skipping vertical lines (slope = infinity")
                continue
            
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)
            
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    return lane_lines

def make_points(frame, line):
    height, width, _ = frame.shape
    
    slope, intercept = line
    
    y1 = height  # bottom of the frame
    y2 = int(y1 / 2)  # make points from middle of the frame down
    
    if slope == 0:
        slope = 0.1
        
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    
    return [[x1, y1, x2, y2]]

def display_lines(frame, lines, line_color=(0, 255, 0), line_width=6):
    line_image = np.zeros_like(frame)
    
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
                
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    
    return line_image


def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5 ):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape
    
    steering_angle_radian = steering_angle / 180.0 * math.pi
    
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)
    
    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)
    
    return heading_image

def get_steering_angle(frame, lane_lines):
    
    height,width,_ = frame.shape
    
    if len(lane_lines) == 2:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        mid = int(width / 2)
        x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height / 2)
        
    elif len(lane_lines) == 1:
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
        y_offset = int(height / 2)
        
    elif len(lane_lines) == 0:
        x_offset = 0
        y_offset = int(height / 2)
        
    angle_to_mid_radian = math.atan(x_offset / y_offset)
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  
    steering_angle = angle_to_mid_deg + 90
    
    return steering_angle

video = cv2.VideoCapture(0)
video.set(cv2.CAP_PROP_FRAME_WIDTH,320)
video.set(cv2.CAP_PROP_FRAME_HEIGHT,240)

time.sleep(1)

##fourcc = cv2.VideoWriter_fourcc(*'XVID')
##out = cv2.VideoWriter('Original15.avi',fourcc,10,(320,240))
##out2 = cv2.VideoWriter('Direction15.avi',fourcc,10,(320,240))

lastTime = 0
lastError = 0

#proportional and derivative constants
kp = 0.05
kd = kp * 0.65

#variables used for stop sign timing
i=0
j=0
frame_count = 0
frame_num = -20

#initial speed
throttle_speed = 7.88

#lists to be extracted to make graphs
frames = []
errors = []
steering_pwms = []
throttle_pwms = []
p_response = []
d_response = []

while frame_count < 500:
    ret,frame = video.read()
    i += 1
    frames.append(i)
    frame = cv2.flip(frame,-1)
    frame = cv2.resize(frame, (60, 40)) #resize camera feed
    frame_count += 1
    
    #set camera
    #cv2.imshow("original",frame)
    edges = detect_edges(frame)
    roi = region_of_interest(edges)
    line_segments = detect_line_segments(roi)
    lane_lines = average_slope_intercept(frame,line_segments)
    lane_lines_image = display_lines(frame,lane_lines)
    steering_angle = get_steering_angle(frame, lane_lines)
    heading_image = display_heading_line(lane_lines_image,steering_angle)
    cv2.imshow("heading line",heading_image)
    
    #set throttle
    PWM.set_duty_cycle(throttlePin, throttle_speed)
    
    #detect stop sign every 3 frames and past 20 frames from last stop
    if i % 3 == 0 and i >= frame_num + 20:
        stop = detect_stopsign(frame)
        if stop == True:
          throttle_speed = 7.905 #give speed boost after stop
          print("stopped")
          frame_num = i
          j+=1
          PWM.set_duty_cycle(throttlePin, 7.5)
          time.sleep(5)
          if j == 3:
            break  #end script after three stops
          PWM.set_duty_cycle(throttlePin, throttle_speed)
          print("started")
    

    #PD controller
    now = time.time()
    dt = now - lastTime

    deviation = steering_angle - 90
    error = abs(deviation)
    errors.append(error)
    steering = 0
    
    derivative = kd * (error - lastError) / dt
    proportional = kp * error
    PD = int(steering + derivative + proportional)
    steering = abs(PD)
    print("steering: " + str(steering))   
    p_response.append(proportional)
    d_response.append(derivative) 
    
    
    #steering logic using PD values
    new_val = 7.5
    if deviation < 9 and deviation > -9:
      print("Not Steering")
      print("Deviation: " + str(deviation))
      new_val = 7.5
      deviation = 0
      error = 0
      PWM.set_duty_cycle(steeringPin, 7.5)
      
    elif deviation > 15:
      print("Steering Right")
      print("Deviation: " + str(deviation))
      new_val = 6 - steering/18
      print("right val: " + str(new_val))
      PWM.set_duty_cycle(steeringPin, new_val)

    elif deviation > 9:
      print("Steering right")
      print("Deviation: " + str(deviation))
      new_val = 6 - steering/20
      print("right val: " + str(new_val))
      PWM.set_duty_cycle(steeringPin, new_val)
      
    elif deviation < -25:
      print("Steering Left")
      print("Deviation: " + str(deviation))
      new_val = 8.6 + steering/13
      print("left val: " + str(new_val))
      throttle_speed = 7.91
      PWM.set_duty_cycle(throttlePin, throttle_speed)
      PWM.set_duty_cycle(steeringPin, new_val)
      
    elif deviation < -20:
      print("Steering Left")
      print("Deviation: " + str(deviation))
      new_val = 8.6 + steering/13
      print("left val: " + str(new_val))
      PWM.set_duty_cycle(steeringPin, new_val)
        
    elif deviation < -9:
      print("Steering Left")
      print("Deviation: " + str(deviation))
      new_val = 8.1 + steering/15
      print("left val: " + str(new_val))
      PWM.set_duty_cycle(steeringPin, new_val)
      
    #append lists
    steering_pwms.append(new_val)
    throttle_pwms.append(throttle_speed)

    lastError = error
    lastTime = time.time()
#   out.write(frame)
#   out2.write(heading_image)

    key = cv2.waitKey(1)
    if key == 27:
        break
    
#extract lists to separate file for analysis
with open("graph_data.txt", "w") as f:
  f.write("frames: \n")
  for item in frames:
    f.write("%s, " % item)
  f.write("\nerrors: \n")
  for item in errors:
    f.write("%s, " % item)
  f.write("\nsteering pwms: \n")
  for item in steering_pwms:
    f.write("%s," % item)
  f.write("\nthrottle pwms: \n")
  for item in throttle_pwms:
    f.write("%s, " % item)
  f.write("\np response: \n")
  for item in p_response:
    f.write("%s, " % item)
  f.write("\nd response: \n")
  for item in d_response:
    f.write("%s, " % item)
  
#close video feed and stop car
video.release()
##out.release()
##out2.release()
cv2.destroyAllWindows()
PWM.set_duty_cycle(throttlePin, 7.5)
PWM.set_duty_cycle(steeringPin, 7.5)

#close pins when finished
PWM.stop("P9_16")
PWM.stop("P9_14")
PWM.cleanup()