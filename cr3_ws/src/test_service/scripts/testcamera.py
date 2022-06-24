import socket
import cv2
import numpy as np
import time
from custom_socket import CustomSocket
import json
image = cv2.imread("zidane.jpg")
print(image.shape)
host = "192.168.8.6"
port = 10001
c = CustomSocket(host,port)
c.clientConnect()
# print(image.tobytes())
while True :
    print("Send")
    msg = c.req(image)
    print(msg)
    time.sleep(10)