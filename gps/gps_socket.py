import socket
import cv2
import socket
# Open the webcam at the primary camera device (0).
capture = cv2.VideoCapture(0)

HOST = "192.168.0.15"
PORT = 65432
soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
soc.connect((HOST, PORT))


while(True):
    ret, frame = capture.read()
    data = soc.recv(1024)
    if len(data) > 0:
        file = open('latlong.dat', 'w')
        print(data)
        file.write(data.decode('utf-8'))
        file.close()
