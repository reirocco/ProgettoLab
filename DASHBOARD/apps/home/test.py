import socket

class SocketConn():

    HOST = '192.168.1.110'  # Standard loopback interface address (localhost)
    PORT = 8888  # Port to listen on (non-privileged ports are > 1023)
    ss = socket.socket(socket.AF_INET, socket.SOCK_STREAM)


    def connect(self):
        data = None
        with self.ss as s:
            s.connect((self.HOST, self.PORT))
            print("client connected")

    def read(self):
        with self.ss as s:
            out = ""
            while 1:
                data = s.recv(1024)
                out = out + data.decode()
                if ('\n' in data.decode()):
                    print(out.strip())
                    out = ""

    def close(self):
        with self.ss as s:
            s.close()


if __name__ == '__main__':
    s = SocketConn()
    s.connect()
    s.read()