import time
import serial

ser = serial.Serial(
    port='/dev/tty.usbmodem14101',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

while (ser.isOpen()):
    line = ser.readline()
    message = {}
    message_segments = line.strip().split(b',')
    #print(message_segments)
    if message_segments[0] == b'$GNGGA':
        # print ("$GNRMC")
        message['type'] = 'GNRMC'
        message['UTC'] = message_segments[1]
        message['Latitude'] = message_segments[2]
        message['Latitude Hem'] = message_segments[3]
        message['Longitude'] = message_segments[4]
        message['Longitude Hem'] = message_segments[5]
        message['height'] = message_segments[9]

        decoded_GNRMC_message = message['Latitude'][:-8] + b' ' + message['Latitude'][-8:] + message[
            'Latitude Hem'] + b',' + message['Longitude'][:-8] + b' ' + message['Longitude'][-8:] + message[
                                    'Longitude Hem']
        print(decoded_GNRMC_message.decode('utf-8'))
        print('height:', message['height'])
        gps_basestation_file = open('basestation_data.dat','w')
        file_message = "{\"height\":" + message['height'].decode('utf-8') + ",\"latlong\": \"" + decoded_GNRMC_message.decode('utf-8') +"\"}"
        gps_basestation_file.write(file_message)
        gps_basestation_file.close()



    time.sleep(0.1)