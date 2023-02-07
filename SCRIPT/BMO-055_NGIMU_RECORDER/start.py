import osc_decoder
import socket
from serial_connection import Serial
# Import writer class from csv module
from csv import writer
from parser import Parser

com = Serial("COM13", 115200)
par = Parser()

list = []
data_list = []

# Send /identify message to strobe all LEDs.  The OSC message is constructed
# from raw bytes as per the OSC specification.  The IP address must be equal to
# the IP address of the target NGIMU.
send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

send_socket.sendto(bytes("/identify\0\0\0,\0\0\0", "utf-8"), ("192.168.1.1", 9000))

#send_socket.sendto(bytes("/rate/euler, 400000f", "utf-8"), ("192.168.1.1", 9000))

# Array of UDP ports to listen to, one per NGIMU.  These ports must be equal to
# the UDP Send Port in the NGIMU settings.  The UDP Send IP Address setting
# must be the computer's IP address.  Both these settings are changed
# automatically when connecting to the NGIMU using the NGIMU GUI.
receive_ports = [8010, 8011, 8012, 8001]

receive_sockets = [socket.socket(socket.AF_INET, socket.SOCK_DGRAM) for _ in range(len(receive_ports))]

if __name__ == '__main__':

    index = 0
    for receive_socket in receive_sockets:
        receive_socket.bind(("", receive_ports[index]))
        index = index + 1
        receive_socket.setblocking(False)

    i = 0
    while True:

        list = []
        for udp_socket in receive_sockets:
            try:
                data, addr = udp_socket.recvfrom(2048)
                json = com.read()

            except socket.error:
                pass
            else:
                for message in osc_decoder.decode(data):

                    if '/euler' in message[1]:
                        print(udp_socket.getsockname(), message)
                        print("COM13", json)
                        bmo_data = par.parse(json)
                        list.append(i)
                        list.append(message[2])
                        list.append(message[3])
                        list.append(message[4])

                        list.append(bmo_data["roll"])
                        list.append(bmo_data["pitch"])
                        list.append(bmo_data["yaw"] - 180)
                        # Open our existing CSV file in append mode
                        # Create a file object for this file
                        with open('result.csv', 'a') as f_object:
                            # Pass this file object to csv.writer()
                            # and get a writer object
                            writer_object = writer(f_object)

                            # Pass the list as an argument into
                            # the writerow()
                            writer_object.writerow(list)

                            # Close the file object
                            f_object.close()
                        i = i + 1
