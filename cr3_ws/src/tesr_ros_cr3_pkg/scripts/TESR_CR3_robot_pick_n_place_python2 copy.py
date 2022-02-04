#!/usr/bin/python2


from dobot_api import dobot_api_dashboard, dobot_api_feedback, MyType
import time
import numpy as np
import threading


# The feedback information about port 30003 is displayed
def CR3_feedback():
    global client_feedback
    while True:
        time.sleep(0.05)
        all = client_feedback.socket_feedback.recv(10240)
        all = all[2:-1]
        data = all[0:1440]
        a = np.frombuffer(data, dtype=MyType)

        #print(hex((a['test_value'][0]))[:13])
        try:
            if hex((a['test_value'][0]))[:13] == '0x123456789ab':
                print("============== Feed Back ===============")
                CR3_endpoint =  np.around(a['tool_vector_actual'], decimals=4)[0]
                print("CR3_endpoint: [x:{0}] , [y:{1}] , [z:{2}] , [rx:{3}] , [ry:{4}] , [rz:{5}]".format(CR3_endpoint[0],CR3_endpoint[1],CR3_endpoint[2],CR3_endpoint[3],CR3_endpoint[4],CR3_endpoint[5]))                            
                CR3_joint = np.around(a['q_actual'], decimals=4)[0]
                print("CR3_joint: [j1:{0}] , [j2:{1}] , [j3:{2}] , [j4:{3}] , [j5:{4}] , [j6:{5}]".format(CR3_joint[0],CR3_joint[1],CR3_joint[2],CR3_joint[3],CR3_joint[4],CR3_joint[5]))
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
        # Call the JointMovJ directive     
        client_feedback.RelMovJ(0, 0, -100, 0, 0, 0)
        time.sleep(3) 
        #close gripper
        client_dashboard.DO(2,0)
        client_dashboard.DO(1,1)
        client_feedback.RelMovJ(0, 0, 100, 0, 0, 0)
        time.sleep(5)

def place():
        #move relative to y 
        client_feedback.RelMovJ(0,100,0, 0, 0, 0)
        #move relative to Z down  
        client_feedback.RelMovJ(0,0,-100, 0, 0, 0)
        #open gripper 
        client_dashboard.DO(2,1)
        client_dashboard.DO(1,0)
        #move relative to z up   
        client_feedback.JointMovJ(0,0,100,0,0,0)
        time.sleep(3) 
        #close gripper
        client_dashboard.DO(2,0)
        client_dashboard.DO(1,1)
        time.sleep(5)

def set_init_pos():
    while dobot_Enable == True:
        client_feedback.JointMovJ(0.39,-0.19,89.83,0.51,-90.42,-44.89)
        time.sleep(3)

def stop_watch():
    global start, dobot_Enable
    if time.time() - start > 10:
        dobot_Enable = False
        
def move():
    global dobot_Enable,client_feedback,client_dashboard
    while dobot_Enable == True:
        set_init_pos()   
        pick()
        place()
                
    client_dashboard.close()
    client_feedback.close()
    print("END program")


if __name__ == "__main__":
    
    dobot_Enable = True
    start = time.time()

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



    t1 = threading.Thread(name ="thread1", target = move)
    t2 = threading.Thread(name ="thread2", target = CR3_feedback)
    t3 = threading.Thread(name ="thread3", target = stop_watch)

    t1.start()
    t2.start()
    t3.start()

    t1.join()
    t2.join()
    t3.join()




