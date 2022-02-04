from dobot_api import dobot_api_dashboard, dobot_api_feedback, MyType
import _thread
import numpy as np
import socket
import cv2 as cv
import time
from custom_socket import CustomSocket
import json

dobot_Enable = True

def CR3_feedback(threadName):
    global dobot_Enable,client_feedback
    while dobot_Enable == True:
        time.sleep(0.05)
        all = client_feedback.socket_feedback.recv(10240)
        data = all[0:1440]
        a = np.frombuffer(data, dtype=MyType)
        try:
            if hex((a['test_value'][0])) == '0x123456789abcdef':
                print("============== Feed Back ===============")
                CR3_endpoint =  np.around(a['Tool_vector_target'], decimals=4)[0]
                print("CR3_endpoint: [x:{0}] , [y:{1}] , [z:{2}] , [rx:{3}] , [ry:{4}] , [rz:{5}]".format(CR3_endpoint[0],CR3_endpoint[1],CR3_endpoint[2],CR3_endpoint[3],CR3_endpoint[4],CR3_endpoint[5]))                            
                CR3_joint = np.around(a['q_target'], decimals=4)[0]
                print("CR3_joint: [j1:{0}] , [j2:{1}] , [j3:{2}] , [j4:{3}] , [j5:{4}] , [j6:{5}]".format(CR3_joint[0],CR3_joint[1],CR3_joint[2],CR3_joint[3],CR3_joint[4],CR3_joint[5]))
                print("========================================")

                CR3_digitalInput =  np.around(a['digital_input_bits'])[0]
                get_bin = lambda x, n: format(x, 'b').zfill(n)
                print("MG400_digitalInput: ",get_bin(CR3_digitalInput,16)) # MSB format
                print("========================================")
        except:
            print("Some error with robot.")
            
            
    client_dashboard.DisableRobot()
    client_feedback.close()
    print('!!!!!! client_feedback END !!!!!!')

def pick():
        #open gripper
        client_dashboard.DO(2,1)
        client_dashboard.DO(1,0)
        time.sleep(3)
        # Call the JointMovJ directive     
        client_feedback.RelMovL(0, 0, -145)
        time.sleep(3) 
        #close gripper
        client_dashboard.DO(2,0)
        client_dashboard.DO(1,1)
        time.sleep(3)
        client_feedback.RelMovL(0, 0, 145)
        time.sleep(3)

def place():
        #move relative to y 
        client_feedback.RelMovL(0,100,0)
        time.sleep(3)
        #move relative to Z down  
        client_feedback.RelMovL(0,0,-145)
        time.sleep(3)
        #open gripper 
        client_dashboard.DO(2,1)
        client_dashboard.DO(1,0)
        time.sleep(3)
        #move relative to z up   
        client_feedback.RelMovL(0,0,145)
        time.sleep(3) 
        #close gripper
        client_dashboard.DO(2,0)
        client_dashboard.DO(1,1)
        time.sleep(3)

def set_init_pos():
        client_feedback.JointMovJ(0.39,-0.19,89.83,0.51,-90.42,-44.89)
        time.sleep(3)

# Enable threads on ports 29999 and 30003
client_dashboard = dobot_api_dashboard('192.168.5.6', 29999)
client_feedback = dobot_api_feedback('192.168.5.6', 30003)


client_dashboard.DisableRobot()
time.sleep(1)

# Remove alarm
client_dashboard.ClearError()
time.sleep(0.5)

# Description The upper function was enabled successfully
client_dashboard.EnableRobot()
time.sleep(0.5)
# Select user and Tool coordinate system 0
client_dashboard.User(0)
client_dashboard.Tool(0)
client_dashboard.DO(2,0)
client_dashboard.DO(1,1)

_thread.start_new_thread(CR3_feedback, ("Thread-1",))

host = "192.168.10.79"
port = 10001

c = CustomSocket(host,port)
c.clientConnect()

cap = cv.VideoCapture(0)
if not cap.isOpened():
    print("Cannot open camera")
    exit()

try:
    while dobot_Enable == True:
        ret, frame = cap.read()    
        if not ret:
                print("Can't receive frame. Exiting...")
                break
        #gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        cv.imshow('frame', frame)

        set_init_pos()
        print("initial position set")

        feat = c.req(frame)
        img_x = feat['width'] 
        img_y = feat['height'] 
        center = [540,360]
        while abs(img_x - center[0] > 50):
                if img_x > center[0]:
                        client_feedback.RelMovL(-5,0,0)
                        continue
                elif img_x < center[0]:
                        client_feedback.RelMovL(5,0,0)
                        continue
        while abs(img_y - center[1] > 50):
                if img_y > center[1]:
                        client_feedback.RelMovL(0,-5,0)
                        continue
                elif img_y < center[1]:
                        client_feedback.RelMovL(0,5,0)
                        continue
        pick()
        place()
        
except KeyboardInterrupt:
    dobot_Enable = False
    client_dashboard.DisableRobot()
    
client_dashboard.close()
client_feedback.close()
print("END program")