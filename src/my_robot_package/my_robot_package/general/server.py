"""
Protocol:
<>#<>#<data>
"""

import socket

PORT = 999

def main():
    with socket.socket() as listen_sock:
        listen_sock.bind(("", PORT))
        listen_sock.listen(1)

        client_sock, client_address = listen_sock.accept()
        print(f"Connected to {client_address}")

        with client_sock:
            client_sock.sendall("hello world".encode())
            print(client_sock.recv(1024).decode())
if __name__ == "__main__":
    main()