# fake_server.py
import socket
import struct
import time

HOST = '0.0.0.0'
PORT = 8688

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)
print("Waiting for connection...")

conn, addr = server_socket.accept()
print(f"Connected by {addr}")

try:
    while True:
        data = struct.pack('f'*6, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
        conn.sendall(data)
        print("Sent:", data)
        time.sleep(0.1)
except:
    conn.close()
    server_socket.close()
