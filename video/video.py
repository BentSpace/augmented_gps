import cv2
import socket
# Open the webcam at the primary camera device (0).
capture = cv2.VideoCapture(0)

HOST = "192.168.0.15"
PORT = 65432
soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
soc.connect((HOST, PORT))


# Check if camera opened successfully
if not capture.isOpened():
    print("Error opening video stream or file")

frame_count = 0
while True:
    # Capture frame-by-frame
    ret, frame = capture.read()
    data = soc.recv(1024)
    if len(data)>0:
        print(data)

    # If frame is read correctly, ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    # Draw a rectangle on frame
    start_point = (50, 50)
    end_point = (150, 150)
    color = (0, 255, 0) # RGB color, here it's green
    thickness = 2
    frame = cv2.rectangle(frame, start_point, end_point, color, thickness)

    # Display the resulting frame
    cv2.imshow('Webcam Feed', frame)

    key = cv2.waitKey(1)
    if key == ord('s'):
        # Save the image when 's' key is pressed.
        cv2.imwrite(f'frame_{frame_count}.png', frame)
        frame_count += 1
    elif key == ord('q'):
        # Quit if 'q' key is pressed
        break

# After the loop release the capture object
capture.release()
# Destroy all the windows
cv2.destroyAllWindows()
