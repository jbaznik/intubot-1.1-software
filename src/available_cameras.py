import cv2
from inputs import devices
from inputs import get_gamepad
import time
index = 0
arr = []
while True:
    try:
        print(devices)
        events = get_gamepad()
        x = 0
        for event in events:
            x += 1
        print(x)
        print(" ")
        print(" ")
    except:
        print("error")

    time.sleep(0.5)
        

'''
    cap = cv2.VideoCapture(index)
    try:
        if cap.getBackendName()=="MSMF":
            arr.append(index)
    except:
        break
    cap.release()
    index += 1

print(arr)
'''