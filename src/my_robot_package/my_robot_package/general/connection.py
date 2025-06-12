from superstrika import SuperStrika
import socket

class Connection(SuperStrika):

    def __init__(self, port):
        super().__init__()

        self.client_addr = None
        self.client_sock = None
        self.port = port

    def server_connect(self):
        with socket.socket() as listen_sock:
            listen_sock.bind(("", self.port))
            listen_sock.listen(1)

            self.client_sock, self.client_addr = listen_sock.accept()
            print(f"Connected to {self.client_addr}")

    def client_connect(self, ip_addr=None, host_name=None):
        with socket.socket() as self.client_sock:
            if not ip_addr and not host_name:
                return
            if host_name:
                self.client_addr = (socket.gethostbyname(host_name), self.port)

            self.client_sock.connect((ip_addr, self.port))
            print(f"Connected to {self.client_addr}")

    @staticmethod
    def protocol_decoder(msg: bytes):
        msg = msg.decode()
        print(msg)

    @staticmethod
    def protocol_encoder(command: int, data: str):
        msg = f"{command}#{data}"
        print(msg)



