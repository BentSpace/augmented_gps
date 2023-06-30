import cv2
import socket
# Open the webcam at the primary camera device (0).
capture = cv2.VideoCapture(0)



# Check if camera opened successfully
if not capture.isOpened():
    print("Error opening video stream or file")

frame_count = 0
while True:
    # Capture frame-by-frame
    ret, frame = capture.read()
    file = open('../gps/latlong.dat', 'rb')
    lat_long_str = file.read(500)
    file.close()
    print(lat_long_str)
    # If frame is read correctly, ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    # Draw a point on frame
    point_position = (50, 50)  # You will need to replace this with your GPS to pixel conversion
    color = (0, 255, 0)  # RGB color, here it's green
    thickness = 5  # You can adjust the thickness to make the point larger or smaller
    frame = cv2.circle(frame, point_position, thickness, color, -1)

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
