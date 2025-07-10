import cv2
cap0=cv2.VideoCapture(0)
try:
    while True:
        ret,frame0=cap0.read()
        if ret:
            print("success")
        else:
            print("failed")
        cv2.waitKey(1)
except:
    print("end")
    cap0.release()