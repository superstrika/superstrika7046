"""
Protocol:
<>#<>#<data>
"""

import socket

SERVER_HOST_NAME = "localhost"

IP_ADDR = socket.gethostbyname(SERVER_HOST_NAME)
print(IP_ADDR)
PORT = 999

def main():
    with socket.socket() as client_sock:
        client_sock.connect((IP_ADDR, PORT))
        print(client_sock.recv(1024).decode())

        client_sock.sendall("hello world too".encode())

if __name__ == "__main__":
    main()