import serial as s
import time
import serial.tools.list_ports as port_list




class Serial:

    def __init__(self, port: str, baud_rate: int):
        self.ser = s.Serial(port, baud_rate, timeout=1)  # ttyACM1 for Arduino board
        self.connected = True

    def read(self):
        readOut = None
        try:
            readOut = self.ser.readline().decode('ascii')
        except:
            pass
        self.ser.flush()  # flush the buffer
        return readOut

    def close(self):
        self.ser.close()


if __name__ == '__main__':
    serial = Serial("COM8", 115200)
    ports = list(port_list.comports())
    for p in ports:
        print(p)
    input("Press any key to continue...")
    while True:
        print(serial.read())
