import cv2
import numpy as np
import ast
import time
import math
import os
import json



# Test mode
TEST_MODE = False
# Test GPS coordinates
TEST_GPS_POSITION = np.array([39.70404879573475, -104.97582286278461, 1614.72])
# Test GPS coordinates start and end
TEST_GPS_POSITION_START = np.array([39.70405987103886, -104.97575293656519, 1614.72])
TEST_GPS_POSITION_END = np.array([39.70396855866599, -104.9758209289261, 1614.72])
# Range North end 39.70405987103886, -104.97575293656519
# Range South end 39.70396855866599, -104.9758209289261
frame_count = 0
counter = 0  # This is for the test mode

D = 7  # distance from the camera to the wall
W = 8  # distance between the two points on the wall

# Calculate FOV in radians
FOV_rad = 2 * math.atan((W/2) / D)

# Convert FOV from radians to degrees
FOV_deg = math.degrees(FOV_rad)

print("FOV_deg: ", FOV_deg)

# Open the webcam at the primary camera device (0).
capture = cv2.VideoCapture(0)

# Camera parameters
# Define the camera's position (latitude, longitude, altitude) in decimal degrees and meters
camera_altitude = 1614.72 # in meters
camera_position = np.array([39.704040301467195, -104.97616054466422, camera_altitude])

# Define the camera's orientation in terms of yaw, pitch, and roll
# For a camera pointing exactly East, the yaw angle (rotation around the vertical axis) should be 90 degrees
# For a camera pointing exactly horizontal, the pitch angle (tilt up/down) should be 0 degrees
# We'll assume no roll angle (rotation around the camera's pointing direction)
camera_orientation = np.array([90.0, 0.0, 0.0])  # in degrees
camera_fov = FOV_deg  # Field of view of the camera in degrees
# Assuming that the vFOV is 2/3 of the hFOV (typical for many cameras)
camera_vfov = 2 / 3 * camera_fov

# Check if camera opened successfully
if not capture.isOpened():
    print("Error opening video stream or file")

frame_count = 0


def get_test_gps_position(counter):
    """
    Returns a test GPS position that moves between the start and end position based on the counter.
    """
    t = (counter % 200) / 200  # Change the divisor to control the speed of back and forth movement

    # Modify the altitude
    altitude_change = 2  # Change in altitude in meters
    altitude_adjustment = altitude_change * (0.5 - abs(t - 0.5)) * 2  # Altitude change is a sinusoidal function

    if t <= 0.5:  # First half of the cycle
        t = 2*t  # Scale t to the range [0, 1]
        position = (TEST_GPS_POSITION_START * (1 - t) + TEST_GPS_POSITION_END * t)
        position[2] += altitude_adjustment
    else:  # Second half of the cycle
        t = 2*(t - 0.5)  # Scale t to the range [0, 1]
        position = (TEST_GPS_POSITION_END * (1 - t) + TEST_GPS_POSITION_START * t)
        position[2] += altitude_adjustment

    return position



def dms_to_dd(dms):
    """
    Converts degrees minutes seconds format to decimal degrees
    """
    degrees, minutes = map(float, dms[:-1].split())
    direction = dms[-1]

    dd = degrees + minutes / 60.

    if direction in ('S','W'):
        dd *= -1

    return dd


def calculate_bearing(pointA, pointB):
    """
    Calculates the bearing from point A to point B.

    Arguments:
    pointA, pointB: numpy arrays representing the points, in the format [latitude, longitude]

    Returns:
    bearing: float representing the bearing from point A to point B, in degrees
    """
    lat1, lon1 = np.radians(pointA[:2])
    lat2, lon2 = np.radians(pointB[:2])

    delta_lon = lon2 - lon1

    x = np.cos(lat2) * np.sin(delta_lon)
    y = np.cos(lat1) * np.sin(lat2) - np.sin(lat1) * np.cos(lat2) * np.cos(delta_lon)

    bearing = np.degrees(np.arctan2(x, y))

    # normalize to 0-360
    return (bearing + 360) % 360


while True:
    print('\n')
    # Capture frame-by-frame
    ret, frame = capture.read()

    if not TEST_MODE:
        # Normal mode

        #Read GPS Base Station Coordinates and Height from file

        with open('../gps/basestation_data.dat', 'r') as file:
            base_station_data_str = file.read()

        base_station_data_dict = json.loads(base_station_data_str)

        print(base_station_data_dict)
        print(base_station_data_dict['latlong'])


        # Read GPS coordinates from file
        with open('../gps/latlong.dat', 'r') as file:
            data_str = file.read()

        # print("data_str: ", data_str)
        # Parse the data

        json_message = json.loads(data_str)

        latlong_str = json_message['latlong']
        height_str = str(json_message['height'])
        #height_str, latlong_str = data_str[1:-1].split(",'latlong':")
        height = float(height_str)  # Split by ":" and take the second element
        latlong = latlong_str.strip("'")  # Remove the quotes

        # Now split latlong to get latitude and longitude
        latitude, longitude = latlong.split(',')
        latitude = dms_to_dd(latitude.strip())  # Remove leading/trailing spaces
        longitude = dms_to_dd(longitude.strip())  # Remove leading/trailing spaces

        height = float(height)

        gps_position = np.array([latitude, longitude, height])

    else:
        gps_position = get_test_gps_position(counter)
        counter += 1
        print("TEST_MODE gps_position: ", gps_position)

    print("gps_position: ", gps_position)
    # Calculate relative position of the GPS point in camera coordinates
    relative_position = gps_position - camera_position
    print("relative_position: ", relative_position)

    # Calculate distance and angle to the GPS point
    distance = np.linalg.norm(relative_position)
    print("distance: ", distance)
    angle = np.arctan2(relative_position[1], relative_position[0])
    print("angle: ", angle)

    # Calculate the bearing from the camera to the GPS point
    bearing = calculate_bearing(camera_position, gps_position)

    # Subtract the camera's yaw angle to get the relative angle
    relative_angle = bearing - camera_orientation[0]  # We're only interested in yaw for now

    # Normalize the relative angle to the range [-180, 180]
    relative_angle = (relative_angle + 180) % 360 - 180
    print("relative_angle",relative_angle)

    # Scale the relative angle to the field of view and convert to pixel coordinates
    pixel_x = int(frame.shape[1] * (relative_angle / camera_fov + 0.5))
    print("pixel_x: ", pixel_x)

    # Calculate distance and angles to the GPS point
    distance_2d = np.linalg.norm(relative_position[:2])  # Distance without considering altitude difference
    print("distance_2d: ", distance_2d)
    vertical_angle = np.degrees(np.arctan2(relative_position[2],
                                           distance_2d))  # This is the vertical angle, with 0 degrees being level with the camera
    print("vertical_angle: ", vertical_angle)

    # Subtract the camera's pitch angle to get the relative vertical angle
    relative_vertical_angle = vertical_angle - camera_orientation[1]  # We're only interested in pitch for now

    # Normalize the relative vertical angle to the range [-180, 180]
    relative_vertical_angle = (relative_vertical_angle + 180) % 360 - 180
    print("relative_vertical_angle", relative_vertical_angle)

    # Scale the relative vertical angle to the vFOV and convert to pixel coordinates
    pixel_y = int(frame.shape[0] * (1 - (relative_vertical_angle + camera_vfov / 2) / camera_vfov))

    # Clamp pixel_y to be within the frame
    pixel_y = max(0, min(frame.shape[0] - 1, pixel_y))
    print("pixel_y: ", pixel_y)

    # Draw a point on the frame
    color = (0, 255, 0)  # RGB color, here it's green
    thickness = 5  # You can adjust the thickness to make the point larger or smaller
    frame = cv2.circle(frame, (pixel_x, pixel_y), thickness, color, -1)
    print("frame.shape: ", frame.shape)

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
    print('\n')
    time.sleep(1)
# After the loop release the capture object
capture.release()
# Destroy all the windows
cv2.destroyAllWindows()

