import cv2
from matplotlib import pyplot as plt

cap = cv2.VideoCapture(0)


if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

while True:
    ret, frame = cap.read()

    cv2.imshow('Webcam', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

#cv2.imwrite('photo.jpg', frame)
#print(ret)
#cap.release()

#plt.imshow(frame)

#while True:
#    

#    cv2.imshow("USB Camera", frame)
#