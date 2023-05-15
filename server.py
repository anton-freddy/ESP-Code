import socket

HOST = '192.168.1.3'  # Change to your host IP address
PORT = 8080            # Change to your desired port number

print("TEST")

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()
    print('Connected by', addr)
    while True:
        data = conn.recv(1024)
        if not data:
            break
        print('Received from client:', data)
        conn.sendall(b'Received: ' + data)
    conn.close()