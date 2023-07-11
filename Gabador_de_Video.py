import cv2

camara=cv2.VideoCapture(0)
video=cv2.VideoWriter("video.mov", cv2.VideoWriter_fourcc(*"MJPG"),60.0, (640,480))
while True:
    ret,frame=camara.read()
    if ret:
        cv2.imshow("video",frame) 
        video.write(frame)
    if cv2.waitKey(1) == ord("s"):
        break
camara.release()
video.release()
cv2.destroyAllWindows()