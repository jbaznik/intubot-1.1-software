import cv2
import os

vid_path_bad = 'C:/Users/was/SynologyDrive/01-data/intuBot/1/1.mp4'
print(vid_path_bad)
vid_path = r'C:/Users/was/SynologyDrive/01-data/intuBot/1/1.mp4'  # notice the r
print(vid_path)

if not os.path.exists(vid_path):  # good to always check we can find the file
    print('file {} not found'.format(vid_path))
    exit(-1)

a = cv2.VideoCapture(vid_path)

while (True):
    success, frame = a.read()
    if success:  # frame read successfully
        cv2.imshow('show', frame)
        k = cv2.waitKey(1)
        if k == ord('q'):
            break

cv2.destroyAllWindows()